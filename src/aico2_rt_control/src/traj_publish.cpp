/**
 * @file traj_publish.cpp
 * @brief Move one joint through the RT path, with no ROS involved.
 *
 * The first real motion test. rt_server is proven to hold position at 1 kHz,
 * but nothing had yet published a trajectory, so the store-sample-stream chain
 * was only tested against a fake robot. This publishes a small single-joint
 * move into the same shared memory the ROS bridge will use, which isolates the
 * RT path from ROS entirely: if the arm moves correctly here, every remaining
 * problem is a ROS problem.
 *
 * It generates a quintic ease, so the trajectory starts and ends at rest with
 * zero acceleration -- the same shape MoveIt produces after Ruckig, and what
 * the validator in traj_ingest.hpp requires.
 *
 * Deliberately awkward to fire by accident: nothing moves without --yes-move,
 * and the travel is capped unless that cap is explicitly raised.
 *
 *   ./traj_publish --list                              # show state, move nothing
 *   ./traj_publish --joint 6 --degrees 3 --yes-move    # 3 deg over 3 s
 *   ./traj_publish --joint 6 --degrees -5 --seconds 6 --yes-move
 *
 * rt_server must already be running against the arm.
 */

#include "aico2_rt_control/traj_ingest.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

constexpr double kDeg2Rad = 0.017453292519943295;
constexpr double kRad2Deg = 57.29577951308232;
/** Travel cap, so a typo cannot ask for a large move. */
constexpr double kMaxDegreesDefault = 15.0;

struct Options {
    std::string shm_name = aico2_rt::kDefaultShmName;
    int joint = -1;
    double degrees = 0.0;
    double seconds = 3.0;
    int points = 24;        // MoveIt's typical density
    double max_degrees = kMaxDegreesDefault;
    bool yes_move = false;
    bool list_only = false;
};

bool ParseArgs(int argc, char** argv, Options& o)
{
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const bool next = (i + 1 < argc);
        if (a == "--list") {
            o.list_only = true;
        } else if (a == "--yes-move") {
            o.yes_move = true;
        } else if (a == "--shm" && next) {
            o.shm_name = argv[++i];
        } else if (a == "--joint" && next) {
            o.joint = std::atoi(argv[++i]);
        } else if (a == "--degrees" && next) {
            o.degrees = std::atof(argv[++i]);
        } else if (a == "--seconds" && next) {
            o.seconds = std::atof(argv[++i]);
        } else if (a == "--points" && next) {
            o.points = std::atoi(argv[++i]);
        } else if (a == "--max-degrees" && next) {
            o.max_degrees = std::atof(argv[++i]);
        } else {
            std::printf(
                "usage: %s [--list] [--joint N --degrees D --yes-move] [options]\n"
                "  --list            print the server's state and limits, move nothing\n"
                "  --joint N         joint index (see --list; 0 and 1 are the waist)\n"
                "  --degrees D       travel, signed\n"
                "  --seconds S       duration (default 3)\n"
                "  --points N        waypoints (default 24)\n"
                "  --max-degrees D   raise the travel cap (default %.0f)\n"
                "  --yes-move        required; without it nothing is published\n"
                "  --shm NAME        shared memory name\n",
                argv[0], kMaxDegreesDefault);
            return false;
        }
    }
    return true;
}

aico2_rt::Shm* OpenShm(const std::string& name, int& fd_out)
{
    const int fd = shm_open(name.c_str(), O_RDWR, 0600);
    if (fd < 0) {
        std::fprintf(stderr, "shm_open(%s): %s\n  Is rt_server running?\n", name.c_str(),
            std::strerror(errno));
        return nullptr;
    }
    void* raw = mmap(nullptr, sizeof(aico2_rt::Shm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (raw == MAP_FAILED) {
        std::fprintf(stderr, "mmap: %s\n", std::strerror(errno));
        close(fd);
        return nullptr;
    }
    fd_out = fd;
    return static_cast<aico2_rt::Shm*>(raw);
}

}  // namespace

int main(int argc, char** argv)
{
    using namespace aico2_rt;

    Options opt;
    if (!ParseArgs(argc, argv, opt)) {
        return 1;
    }
    setvbuf(stdout, nullptr, _IONBF, 0);

    int fd = -1;
    Shm* shm = OpenShm(opt.shm_name, fd);
    if (shm == nullptr) {
        return 1;
    }
    if (shm->magic != kShmMagic || shm->version != kShmVersion) {
        std::fprintf(stderr, "shared memory is not a v%u aico2_rt mapping\n", kShmVersion);
        return 1;
    }

    // The RT heartbeat is the cycle counter, so a server that is running moves
    // it ~1000 times a second. If it does not move at all over 200 ms, nothing
    // is driving the robot and publishing would achieve nothing.
    const std::uint64_t hb0 = shm->rt_heartbeat.load();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (shm->rt_heartbeat.load() == hb0) {
        std::fprintf(stderr, "rt_server's heartbeat is not advancing -- is it running?\n");
        return 1;
    }

    const std::uint32_t dof = shm->dof;
    Measured meas{};
    RtStatus st{};
    if (!ReadMeasured(*shm, meas) || !ReadStatus(*shm, st)) {
        std::fprintf(stderr, "could not read a coherent state from shared memory\n");
        return 1;
    }

    std::printf("\nserver: dof %u, external axes %u, waist %s, exec state %u\n", dof,
        shm->n_external, shm->control_waist ? "COMMANDED" : "pinned", st.exec_state);
    std::printf("%-10s", "index");
    for (std::uint32_t j = 0; j < dof; ++j) {
        std::printf(" %8u", j);
    }
    std::printf("\n%-10s", "q (deg)");
    for (std::uint32_t j = 0; j < dof; ++j) {
        std::printf(" %8.2f", meas.q[j] * kRad2Deg);
    }
    std::printf("\n%-10s", "min");
    for (std::uint32_t j = 0; j < dof; ++j) {
        std::printf(" %8.1f", shm->limits.q_min[j] * kRad2Deg);
    }
    std::printf("\n%-10s", "max");
    for (std::uint32_t j = 0; j < dof; ++j) {
        std::printf(" %8.1f", shm->limits.q_max[j] * kRad2Deg);
    }
    std::printf("\n\n");

    if (opt.list_only) {
        return 0;
    }
    if (opt.joint < 0 || static_cast<std::uint32_t>(opt.joint) >= dof) {
        std::fprintf(stderr, "--joint must be 0..%u\n", dof - 1);
        return 1;
    }
    if (std::fabs(opt.degrees) > opt.max_degrees) {
        std::fprintf(stderr, "refusing %.1f deg: over the %.1f deg cap. Raise --max-degrees"
                             " deliberately if that is really wanted.\n",
            opt.degrees, opt.max_degrees);
        return 1;
    }
    if (opt.seconds <= 0.2 || opt.points < 4 || opt.points > static_cast<int>(kMaxPoints)) {
        std::fprintf(stderr, "--seconds must exceed 0.2 and --points be 4..%zu\n", kMaxPoints);
        return 1;
    }

    // Quintic ease: zero velocity AND zero acceleration at both ends, which is
    // what the validator requires and what the sampler reproduces exactly.
    const auto j = static_cast<std::uint32_t>(opt.joint);
    const double delta = opt.degrees * kDeg2Rad;
    const double T = opt.seconds;
    const auto n = static_cast<std::uint32_t>(opt.points);
    std::vector<Point> pts(n);
    for (std::uint32_t i = 0; i < n; ++i) {
        const double u = static_cast<double>(i) / (n - 1);
        const double e = 10 * u * u * u - 15 * u * u * u * u + 6 * u * u * u * u * u;
        const double de = (30 * u * u - 60 * u * u * u + 30 * u * u * u * u) / T;
        const double dde = (60 * u - 180 * u * u + 120 * u * u * u) / (T * T);
        Point& p = pts[i];
        p.t = T * u;
        for (std::uint32_t k = 0; k < kMaxDof; ++k) {
            p.q[k] = (k < dof) ? meas.q[k] : 0.0;
            p.dq[k] = 0.0;
            p.ddq[k] = 0.0;
        }
        p.q[j] = meas.q[j] + delta * e;
        p.dq[j] = delta * de;
        p.ddq[j] = delta * dde;
    }

    IngestConfig cfg;
    cfg.dof = dof;
    cfg.n_external = shm->n_external;
    cfg.allow_waist_motion = (shm->control_waist != 0u);
    cfg.limits = shm->limits;

    const RejectReason why = ValidateTrajectory(pts.data(), n, meas.q, cfg);
    if (why != RejectReason::kNone) {
        std::fprintf(stderr, "this trajectory would be rejected: %s\n", RejectReasonName(why));
        return 1;
    }
    std::printf("joint %u: %.2f -> %.2f deg over %.1f s, %u points. Validated.\n", j,
        meas.q[j] * kRad2Deg, (meas.q[j] + delta) * kRad2Deg, T, n);

    if (!opt.yes_move) {
        std::printf("\nNot published: --yes-move was not given. The arm WILL move with it.\n");
        return 0;
    }

    const std::uint32_t slot = PickFreeSlot(*shm);
    FillSlot(shm->slots[slot], 1, pts.data(), n, dof);
    const std::uint64_t seq = PublishSlot(*shm, slot);
    std::printf("published slot %u, seq %llu\n", slot,
        static_cast<unsigned long long>(seq));

    // Adoption happens on the server's next cycle, so ~1 ms.
    for (int i = 0; i < 500 && shm->adopted_seq.load() < seq; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (shm->adopted_seq.load() < seq) {
        std::fprintf(stderr, "the server did not adopt it within 500 ms\n");
        return 1;
    }

    const auto deadline = std::chrono::steady_clock::now()
                          + std::chrono::milliseconds(static_cast<long>((T + 5.0) * 1000));
    std::uint32_t last = 0xFFFFFFFFu;
    while (std::chrono::steady_clock::now() < deadline) {
        if (ReadStatus(*shm, st)) {
            if (st.exec_state != last) {
                std::printf("  state %u  t %.2f s  q[%u] %.2f deg\n", st.exec_state, st.traj_time,
                    j, st.cmd_q[j] * kRad2Deg);
                last = st.exec_state;
            }
            if (st.exec_state == static_cast<std::uint32_t>(ExecState::kFinished)
                || st.exec_state == static_cast<std::uint32_t>(ExecState::kAborted)
                || st.exec_state == static_cast<std::uint32_t>(ExecState::kRejected)) {
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    ReadMeasured(*shm, meas);
    ReadStatus(*shm, st);
    const double want = (pts[n - 1].q[j]) * kRad2Deg;
    const double got = meas.q[j] * kRad2Deg;
    std::printf("\nfinal state %u  (3 = rejected, 4 = aborted, 2 = finished)\n", st.exec_state);
    std::printf("commanded %.3f deg, measured %.3f deg, error %.3f deg\n", want, got, got - want);
    if (st.clamped) {
        std::printf("CLAMPED during execution: joint %u, %s\n", st.clamp_joint,
            ClampKindName(static_cast<ClampKind>(st.clamp_kind)));
    } else {
        std::printf("clamped during execution: no\n");
    }
    std::printf("loop: cycles %llu, missed %llu, max period %.3f ms\n",
        static_cast<unsigned long long>(st.cycles),
        static_cast<unsigned long long>(st.missed_deadlines), st.max_period_sec * 1e3);

    munmap(shm, sizeof(Shm));
    close(fd);
    return st.exec_state == static_cast<std::uint32_t>(ExecState::kFinished) ? 0 : 1;
}
