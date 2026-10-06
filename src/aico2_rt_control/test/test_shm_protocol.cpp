// Verifies the shared-memory contract: slot rotation, the seqlock under a
// concurrent 1 kHz-ish writer, and that the layout really works across a
// process boundary rather than only across threads.

#include "aico2_rt_control/shm_protocol.hpp"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

using namespace aico2_rt;

namespace {

int g_failures = 0;

void Check(bool ok, const char* what)
{
    if (!ok) {
        ++g_failures;
        std::printf("  FAIL  %s\n", what);
    }
}

void TestSlotRotationNeverCollides()
{
    std::printf("the writer never picks a slot in use\n");
    auto* shm = new Shm{};
    shm->published_slot.store(kNoSlot);
    shm->reading_slot.store(kNoSlot);

    // Walk every reachable (published, reading) pair, including the equal case
    // and the "nothing yet" sentinel.
    for (std::uint32_t pub = 0; pub <= kNumSlots; ++pub) {
        for (std::uint32_t rd = 0; rd <= kNumSlots; ++rd) {
            const std::uint32_t p = (pub == kNumSlots) ? kNoSlot : pub;
            const std::uint32_t r = (rd == kNumSlots) ? kNoSlot : rd;
            shm->published_slot.store(p);
            shm->reading_slot.store(r);
            const std::uint32_t free_slot = PickFreeSlot(*shm);
            Check(free_slot < kNumSlots, "picked slot is in range");
            Check(free_slot != p || p == kNoSlot, "picked slot is not the published one");
            Check(free_slot != r || r == kNoSlot, "picked slot is not the one being read");
        }
    }
    delete shm;
}

void TestPublishSeqIncreases()
{
    std::printf("publish_seq increases and the index follows it\n");
    auto* shm = new Shm{};
    shm->published_slot.store(kNoSlot);
    shm->reading_slot.store(kNoSlot);
    std::uint64_t last = 0;
    for (int i = 0; i < 50; ++i) {
        const std::uint32_t slot = PickFreeSlot(*shm);
        const std::uint64_t seq = PublishSlot(*shm, slot);
        Check(seq > last, "sequence strictly increases");
        last = seq;
        Check(shm->published_slot.load() == slot, "published index matches");
        // Pretend the RT side adopts it, as it would next cycle.
        MarkReading(*shm, slot);
        AcknowledgeSeq(*shm, seq);
        Check(shm->adopted_seq.load() == seq, "acknowledgement observed");
    }
    delete shm;
}

/** The point of a seqlock is that a reader never observes a half-written
 *  struct. Write a state whose fields must all agree, hammer it, and check. */
void TestSeqlockNeverTears()
{
    std::printf("the seqlock never yields a torn state\n");
    auto* shm = new Shm{};
    std::atomic<bool> stop{false};
    std::atomic<long> reads{0}, torn{0}, retries{0};

    std::thread writer([&] {
        for (std::uint64_t k = 1; !stop.load(std::memory_order_relaxed); ++k) {
            BeginStateWrite(*shm);
            // Every field derived from k, so any mixture of two generations is
            // detectable. Written field by field, as the RT task would.
            shm->state.dof = 9;
            shm->state.cycles = k;
            shm->state.stamp_mono = static_cast<double>(k);
            for (std::size_t j = 0; j < kMaxDof; ++j) {
                shm->state.q[j] = static_cast<double>(k);
                shm->state.dq[j] = static_cast<double>(k);
                shm->state.tau[j] = static_cast<double>(k);
            }
            shm->state.active_id = k;
            EndStateWrite(*shm);
        }
    });

    State out{};
    while (reads.load() < 200000) {
        if (!ReadState(*shm, out)) {
            retries.fetch_add(1);
            continue;
        }
        reads.fetch_add(1);
        const double k = out.stamp_mono;
        bool consistent = (out.cycles == static_cast<std::uint64_t>(k))
                       && (out.active_id == static_cast<std::uint64_t>(k));
        for (std::size_t j = 0; j < kMaxDof && consistent; ++j) {
            consistent = out.q[j] == k && out.dq[j] == k && out.tau[j] == k;
        }
        if (!consistent) {
            torn.fetch_add(1);
        }
    }
    stop.store(true);
    writer.join();

    // The retry count says nothing about production: this writer spins as fast
    // as it can, whereas the RT task writes once per millisecond and leaves the
    // seqlock even for essentially the whole period. Torn reads are the number
    // that matters, and it must be zero.
    std::printf("  %ld coherent reads, %ld torn, %ld retries (writer unthrottled)\n",
        reads.load(), torn.load(), retries.load());
    Check(torn.load() == 0, "no torn reads");
    Check(reads.load() > 0, "reads actually happened");
    delete shm;
}

void TestReadStateReportsFailureMidWrite()
{
    std::printf("a read during a write reports failure rather than garbage\n");
    auto* shm = new Shm{};
    BeginStateWrite(*shm);  // leave the sequence odd
    State out{};
    Check(!ReadState(*shm, out, 4), "read fails while a write is open");
    EndStateWrite(*shm);
    Check(ReadState(*shm, out, 4), "read succeeds once the write closes");
    delete shm;
}

/** The layout has to work in a second process, not just a second thread: a
 *  stray pointer or a non-lock-free atomic would only show up here. */
void TestAcrossRealProcesses()
{
    std::printf("the mapping works across a fork with real shared memory\n");
    const char* name = "/aico2_rt_control_test";
    shm_unlink(name);
    const int fd = shm_open(name, O_CREAT | O_RDWR, 0600);
    Check(fd >= 0, "shm_open succeeded");
    if (fd < 0) {
        return;
    }
    Check(ftruncate(fd, sizeof(Shm)) == 0, "ftruncate to the mapping size");
    void* raw = mmap(nullptr, sizeof(Shm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    Check(raw != MAP_FAILED, "mmap succeeded");
    if (raw == MAP_FAILED) {
        close(fd);
        shm_unlink(name);
        return;
    }
    auto* shm = new (raw) Shm{};
    shm->magic = kShmMagic;
    shm->version = kShmVersion;
    shm->published_slot.store(kNoSlot);
    shm->reading_slot.store(kNoSlot);

    const pid_t pid = fork();
    Check(pid >= 0, "fork succeeded");
    if (pid == 0) {
        // Child: fill a slot, publish, then mirror the parent's state back.
        const std::uint32_t slot = PickFreeSlot(*shm);
        Slot& s = shm->slots[slot];
        s.id = 7;
        s.n_points = 3;
        s.n_joints = 9;
        for (std::uint32_t i = 0; i < 3; ++i) {
            s.points[i].t = 0.5 * i;
            for (std::uint32_t j = 0; j < 9; ++j) {
                s.points[i].q[j] = 100.0 * i + j;
            }
        }
        PublishSlot(*shm, slot);
        shm->bridge_heartbeat.store(12345);
        _exit(0);
    }
    int status = 0;
    waitpid(pid, &status, 0);
    Check(WIFEXITED(status) && WEXITSTATUS(status) == 0, "child exited cleanly");

    // Parent sees everything the child wrote, including through the atomics.
    Check(shm->bridge_heartbeat.load() == 12345, "atomic crossed the boundary");
    const std::uint32_t pub = shm->published_slot.load();
    Check(pub < kNumSlots, "a slot was published");
    if (pub < kNumSlots) {
        const Slot& s = shm->slots[pub];
        Check(s.id == 7, "slot id crossed the boundary");
        Check(s.n_points == 3 && s.n_joints == 9, "slot dimensions crossed");
        bool points_ok = true;
        for (std::uint32_t i = 0; i < 3 && points_ok; ++i) {
            points_ok = s.points[i].t == 0.5 * i;
            for (std::uint32_t j = 0; j < 9 && points_ok; ++j) {
                points_ok = s.points[i].q[j] == 100.0 * i + j;
            }
        }
        Check(points_ok, "trajectory points crossed intact");
    }
    munmap(raw, sizeof(Shm));
    close(fd);
    shm_unlink(name);
}

void TestSizeIsReasonable()
{
    std::printf("the mapping is a sane size\n");
    const double mb = static_cast<double>(sizeof(Shm)) / (1024.0 * 1024.0);
    std::printf("  sizeof(Shm) = %zu bytes (%.2f MiB), Slot = %zu, Point = %zu\n",
        sizeof(Shm), mb, sizeof(Slot), sizeof(Point));
    Check(mb < 32.0, "mapping under 32 MiB");
    Check(sizeof(Point) == (1 + 3 * kMaxDof) * sizeof(double), "Point has no padding");
}

}  // namespace

int main()
{
    std::printf("=== shm_protocol ===\n");
    TestSlotRotationNeverCollides();
    TestPublishSeqIncreases();
    TestSeqlockNeverTears();
    TestReadStateReportsFailureMidWrite();
    TestAcrossRealProcesses();
    TestSizeIsReasonable();
    std::printf("%s (%d failures)\n", g_failures == 0 ? "PASS" : "FAIL", g_failures);
    return g_failures == 0 ? 0 : 1;
}
