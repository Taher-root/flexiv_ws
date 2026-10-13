// rt_server with the robot replaced by a model. Needs no RDK and no hardware.
//
// Exists so the two-process handshake can be exercised for real -- a separate
// process, the same shared memory, the same executor, at a true 1 kHz -- before
// anything is pointed at an arm. traj_publish and the ROS bridge cannot tell it
// from the real server, which makes it the right place to find protocol bugs.
//
//   ./fake_server --seconds 30 &
//   ./traj_publish --joint 6 --degrees 3 --yes-move

#include "aico2_rt_control/rt_executor.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <new>
#include <string>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>

using namespace aico2_rt;

namespace {

std::atomic<bool> g_stop{false};
void OnSignal(int) { g_stop.store(true); }

constexpr std::uint32_t kDof = 9;
constexpr std::uint32_t kExternal = 2;

/** Tracks commands exactly, which is the useful default: any error seen
 *  downstream is then the pipeline's, not the model's. */
struct ModelRobot {
    double q[kMaxDof]{};
    double dq[kMaxDof]{};
    bool faulted = false;

    bool fault() const { return faulted; }
    bool operational() const { return !faulted; }
    void Stream(const double* cq, const double* cdq, const double*)
    {
        for (std::uint32_t j = 0; j < kDof; ++j) {
            q[j] = cq[j];
            dq[j] = cdq[j];
        }
    }
};

// The clock from shm_protocol.hpp, not a process-relative one: servo_publish
// stamps targets with it from another process, and the tracker compares the two
// directly. See MonotonicSeconds()' comment there.
double Mono()
{
    return MonotonicSeconds();
}

}  // namespace

int main(int argc, char** argv)
{
    std::string name = kDefaultShmName;
    double seconds = 30.0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--shm" && i + 1 < argc) {
            name = argv[++i];
        } else if (a == "--seconds" && i + 1 < argc) {
            seconds = std::atof(argv[++i]);
        }
    }
    setvbuf(stdout, nullptr, _IONBF, 0);
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    shm_unlink(name.c_str());
    const int fd = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0600);
    if (fd < 0) {
        std::fprintf(stderr, "shm_open: %s\n", std::strerror(errno));
        return 1;
    }
    if (ftruncate(fd, sizeof(Shm)) != 0) {
        std::fprintf(stderr, "ftruncate: %s\n", std::strerror(errno));
        return 1;
    }
    void* raw = mmap(nullptr, sizeof(Shm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (raw == MAP_FAILED) {
        std::fprintf(stderr, "mmap: %s\n", std::strerror(errno));
        return 1;
    }
    auto* shm = new (raw) Shm{};
    shm->magic = kShmMagic;
    shm->version = kShmVersion;
    shm->dof = kDof;
    shm->n_external = kExternal;
    shm->control_waist = 0u;
    shm->published_slot.store(kNoSlot);
    shm->reading_slot.store(kNoSlot);

    // Roughly the real robot's envelope, so a trajectory accepted here would be
    // accepted there.
    Limits limits{};
    const double qmin[kDof] = {-1.526, 0.0436, -2.792, -2.269, -2.967, -1.867, -2.967, -1.396,
        -2.967};
    const double qmax[kDof] = {1.526, 1.526, 2.792, 2.269, 2.967, 2.688, 2.967, 4.538, 2.967};
    const double dqmax[kDof] = {1.047, 1.047, 2.094, 2.094, 2.443, 2.443, 4.887, 4.887, 4.887};
    ModelRobot robot;
    for (std::uint32_t j = 0; j < kDof; ++j) {
        limits.q_min[j] = qmin[j];
        limits.q_max[j] = qmax[j];
        limits.dq_max[j] = dqmax[j];
        limits.ddq_max[j] = 3.0;
        robot.q[j] = 0.5 * (qmin[j] + qmax[j]) * 0.1;  // somewhere innocuous
    }
    robot.q[1] = 0.157;  // above the waist's positive floor
    // Waist pinned, exactly as rt_server does without --control-waist.
    // Mirrors rt_server, including the epsilon: see the comment there on why
    // pinning to an exact value makes the clamp indicator fire on round-off.
    constexpr double kPinEpsilon = 1e-3;
    for (std::uint32_t j = 0; j < kExternal; ++j) {
        limits.q_min[j] = robot.q[j] - kPinEpsilon;
        limits.q_max[j] = robot.q[j] + kPinEpsilon;
        limits.dq_max[j] = kPinEpsilon;
        limits.ddq_max[j] = kPinEpsilon;
    }
    shm->limits = limits;

    RtExecutor<ModelRobot> executor;
    executor.Init(shm, &robot, kDof, limits, robot.q);

    std::printf("fake_server on '%s': dof %u, waist pinned, %.0f s\n", name.c_str(), kDof,
        seconds);

    // A real 1 kHz loop on an ordinary thread. Not RT-scheduled, so the timing
    // is only approximate -- that is fine, since what is under test here is the
    // protocol, not the scheduler.
    std::thread loop([&] {
        const auto period = std::chrono::microseconds(1000);
        auto next = std::chrono::steady_clock::now();
        while (!g_stop.load()) {
            executor.Cycle(Mono());
            next += period;
            std::this_thread::sleep_until(next);
        }
    });

    const double t0 = Mono();
    std::uint64_t ticks = 0;
    while (!g_stop.load() && Mono() - t0 < seconds) {
        BeginMeasuredWrite(*shm);
        shm->measured.stamp_mono = Mono();
        for (std::uint32_t j = 0; j < kDof; ++j) {
            shm->measured.q[j] = robot.q[j];
            shm->measured.dq[j] = robot.dq[j];
            shm->measured.tau[j] = 0.0;
        }
        shm->measured.dof = kDof;
        shm->measured.operational = 1u;
        shm->measured.fault = 0u;
        EndMeasuredWrite(*shm);
        if (++ticks % 200 == 0) {
            std::printf("cycles %llu  state %u  q[6] %.4f\n",
                static_cast<unsigned long long>(executor.cycles()),
                static_cast<unsigned>(executor.state()), robot.q[6]);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    g_stop.store(true);
    loop.join();

    std::printf("\ncycles %llu  missed %llu  mean %.4f ms  max %.4f ms\n",
        static_cast<unsigned long long>(executor.cycles()),
        static_cast<unsigned long long>(executor.missed()), executor.mean_period() * 1e3,
        executor.max_period() * 1e3);
    munmap(raw, sizeof(Shm));
    close(fd);
    shm_unlink(name.c_str());
    return 0;
}
