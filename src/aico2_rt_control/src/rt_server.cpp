/**
 * @file rt_server.cpp
 * @brief The RT half: owns the robot session and the 1 kHz loop. No ROS.
 *
 * Deliberately thin. Everything with a decision in it lives in
 * rt_executor.hpp, which is unit-tested against a fake robot; this file is RDK
 * calls, argument parsing and process lifetime. The split exists because the
 * RDK is a prebuilt binary that cannot be linked on a development machine, so
 * any logic left here would only ever be exercised on hardware.
 *
 * Two processes rather than one because a MoveIt controller links rclcpp and
 * therefore loads ROS 2's Fast-DDS, while the only RDK archive that works on
 * this robot expects Flexiv's vendored version -- see README.md. This half
 * links neither rclcpp nor anything ROS.
 *
 *   ./rt_server Rizon4-063352                      # arm only, runs until Ctrl-C
 *   ./rt_server Rizon4-063352 --control-waist      # also drive the 2 waist axes
 *   ./rt_server Rizon4-063352 --duration 30
 *
 * Preconditions, as for every RDK entry point here: E-stop released, motion bar
 * in Auto (Remote), and no other process holding a session to this arm -- the
 * Python driver must be stopped.
 */

#include "aico2_rt_control/rt_executor.hpp"

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <new>
#include <string>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

std::atomic<bool> g_shutdown{false};
void OnSignal(int) { g_shutdown.store(true); }

/**
 * @brief Adapts flexiv::rdk::Robot to the three methods RtExecutor needs.
 *
 * The command vectors are members, sized once. StreamJointPosition takes
 * const std::vector<double>&, so constructing them per cycle would allocate
 * inside the 1 kHz task; assigning into vectors that are already the right size
 * does not.
 *
 * Note there is no state accessor here. Robot::states() returns a value copy of
 * a struct holding twelve std::vector<double> and therefore allocates, so it
 * belongs in the non-RT thread below, not in the control loop.
 */
class RobotAdapter {
public:
    RobotAdapter(flexiv::rdk::Robot& robot, std::size_t dof)
    : robot_(robot), q_(dof, 0.0), dq_(dof, 0.0), ddq_(dof, 0.0)
    {
    }

    bool fault() const { return robot_.fault(); }
    bool operational() const { return robot_.operational(); }

    void Stream(const double* q, const double* dq, const double* ddq)
    {
        for (std::size_t j = 0; j < q_.size(); ++j) {
            q_[j] = q[j];
            dq_[j] = dq[j];
            ddq_[j] = ddq[j];
        }
        robot_.StreamJointPosition(q_, dq_, ddq_);
    }

private:
    flexiv::rdk::Robot& robot_;
    std::vector<double> q_, dq_, ddq_;
};

double MonotonicSeconds()
{
    using Clock = std::chrono::steady_clock;
    static const Clock::time_point t0 = Clock::now();
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

struct Options {
    std::string robot_sn;
    std::string shm_name = aico2_rt::kDefaultShmName;
    bool control_waist = false;
    double max_acc = 3.0;   // rad/s^2; matches the Python driver's constant
    double duration = 0.0;  // 0 = until signalled
};

bool ParseArgs(int argc, char** argv, Options& opt)
{
    if (argc < 2 || argv[1][0] == '-') {
        std::printf(
            "usage: %s <robot_sn> [options]\n"
            "  --shm NAME          shared memory name (default %s)\n"
            "  --control-waist     also command the external axes. Off by\n"
            "                      default: with it off they are pinned at\n"
            "                      their current position, so a 9-joint\n"
            "                      trajectory cannot move the torso.\n"
            "  --max-acc A         acceleration clamp, rad/s^2 (default 3.0).\n"
            "                      RobotInfo carries no acceleration limit, so\n"
            "                      this cannot be read from the robot.\n"
            "  --duration S        stop after S seconds (default: until Ctrl-C)\n",
            argv[0], aico2_rt::kDefaultShmName);
        return false;
    }
    opt.robot_sn = argv[1];
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        const bool has_next = (i + 1 < argc);
        if (a == "--control-waist") {
            opt.control_waist = true;
        } else if (a == "--shm" && has_next) {
            opt.shm_name = argv[++i];
        } else if (a == "--max-acc" && has_next) {
            opt.max_acc = std::atof(argv[++i]);
        } else if (a == "--duration" && has_next) {
            opt.duration = std::atof(argv[++i]);
        } else {
            std::fprintf(stderr, "unrecognised argument: %s\n", a.c_str());
            return false;
        }
    }
    if (opt.max_acc <= 0.0) {
        std::fprintf(stderr, "--max-acc must be positive\n");
        return false;
    }
    return true;
}

/** Create the mapping fresh. A stale one from a previous run may hold a
 *  published trajectory, which must not be adopted on startup. */
aico2_rt::Shm* CreateShm(const std::string& name, int& fd_out)
{
    shm_unlink(name.c_str());
    const int fd = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0600);
    if (fd < 0) {
        std::fprintf(stderr, "shm_open(%s): %s\n", name.c_str(), std::strerror(errno));
        return nullptr;
    }
    if (ftruncate(fd, sizeof(aico2_rt::Shm)) != 0) {
        std::fprintf(stderr, "ftruncate: %s\n", std::strerror(errno));
        close(fd);
        return nullptr;
    }
    void* raw = mmap(nullptr, sizeof(aico2_rt::Shm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (raw == MAP_FAILED) {
        std::fprintf(stderr, "mmap: %s\n", std::strerror(errno));
        close(fd);
        return nullptr;
    }
    fd_out = fd;
    return new (raw) aico2_rt::Shm{};
}

}  // namespace

int main(int argc, char** argv)
{
    Options opt;
    if (!ParseArgs(argc, argv, opt)) {
        return 1;
    }
    std::cout.setf(std::ios::unitbuf);
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    int shm_fd = -1;
    aico2_rt::Shm* shm = CreateShm(opt.shm_name, shm_fd);
    if (shm == nullptr) {
        return 1;
    }
    shm->magic = aico2_rt::kShmMagic;
    shm->version = aico2_rt::kShmVersion;
    shm->published_slot.store(aico2_rt::kNoSlot);
    shm->reading_slot.store(aico2_rt::kNoSlot);

    int rc = 0;
    try {
        flexiv::rdk::Robot robot(opt.robot_sn);

        if (robot.fault()) {
            std::printf("fault present, clearing\n");
            if (!robot.ClearFault()) {
                std::fprintf(stderr, "ClearFault failed\n");
                throw std::runtime_error("could not clear fault");
            }
        }
        std::printf("enabling; E-stop released and motion bar in Auto (Remote)\n");
        robot.Enable();
        for (int i = 0; i < 30 && !robot.operational(); ++i) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        if (!robot.operational()) {
            throw std::runtime_error("not operational after 30s");
        }

        const auto info = robot.info();
        const auto dof = static_cast<std::uint32_t>(info.DoF);
        const auto ext = static_cast<std::uint32_t>(info.DoF_e);
        if (dof > aico2_rt::kMaxDof) {
            throw std::runtime_error("robot DoF exceeds kMaxDof");
        }
        std::printf("DoF %u (arm %zu, external %zu)\n", dof, info.DoF_m, info.DoF_e);

        // Limits from the robot's own software limits, except acceleration,
        // which RobotInfo does not carry.
        aico2_rt::Limits limits{};
        for (std::uint32_t j = 0; j < dof; ++j) {
            limits.q_min[j] = info.q_min[j];
            limits.q_max[j] = info.q_max[j];
            limits.dq_max[j] = info.dq_max[j];
            limits.ddq_max[j] = opt.max_acc;
        }

        const auto q_now = robot.states().q;  // non-RT, allocation is fine here

        // With the waist off, collapse its position window onto where it is.
        // The executor clamps every cycle, so this pins those axes using the
        // mechanism that is already there rather than a special case in the
        // control loop. The external axes are indices 0..DoF_e-1, confirmed by
        // joint_map_probe.
        if (!opt.control_waist) {
            // A window, not exact equality. The sampler evaluates
            // H0*q0 + H3*q1 where H0 + H3 is 1 analytically but not in
            // floating point, so pinning to a single value leaves the result
            // a ULP outside it and the clamp fires on every cycle. That turns
            // the clamp indicator -- which should mean "a trajectory asked for
            // something the limits forbid" -- into noise. 1e-6 rad is 6e-5
            // degrees: physically nothing, and many orders of magnitude above
            // the round-off.
            constexpr double kPinEpsilon = 1e-6;
            for (std::uint32_t j = 0; j < ext && j < dof; ++j) {
                limits.q_min[j] = q_now[j] - kPinEpsilon;
                limits.q_max[j] = q_now[j] + kPinEpsilon;
                limits.dq_max[j] = kPinEpsilon;
                limits.ddq_max[j] = kPinEpsilon;
            }
            std::printf("waist NOT commanded: axes 0..%u pinned at their current position\n",
                ext ? ext - 1 : 0);
        } else {
            std::printf("waist WILL be commanded (--control-waist)\n");
        }

        // LockExternalAxes requires IDLE, so it must precede SwitchMode.
        if (ext > 0) {
            robot.LockExternalAxes(!opt.control_waist);
        }

        std::printf("holding at %s\n", flexiv::rdk::utility::Vec2Str(q_now).c_str());
        std::printf("switching to RT_JOINT_POSITION\n");
        robot.SwitchMode(flexiv::rdk::Mode::RT_JOINT_POSITION);

        RobotAdapter adapter(robot, dof);
        aico2_rt::RtExecutor<RobotAdapter> executor;
        executor.Init(shm, &adapter, dof, limits, q_now.data());
        // Publish the configuration a writer needs in order to produce a
        // trajectory this server will accept without clamping.
        shm->dof = dof;
        shm->n_external = ext;
        shm->control_waist = opt.control_waist ? 1u : 0u;
        shm->limits = limits;

        // Anything thrown out of the periodic task would cross the scheduler
        // boundary, so it is caught here and reported through a flag the main
        // thread polls. Nothing in the normal path throws.
        std::atomic<bool> task_error{false};
        static char err_text[256] = {};

        flexiv::rdk::Scheduler scheduler;
        scheduler.AddTask(
            [&] {
                try {
                    executor.Cycle(MonotonicSeconds());
                } catch (const std::exception& e) {
                    std::snprintf(err_text, sizeof(err_text), "%s", e.what());
                    task_error.store(true, std::memory_order_release);
                }
            },
            "rt_control", 1, scheduler.max_priority());

        std::printf("scheduler started at 1 kHz; shm '%s'. Ctrl-C to stop.\n",
            opt.shm_name.c_str());
        scheduler.Start();

        // Non-RT thread: publish measured state for the bridge, and watch for
        // reasons to stop. states() allocates, which is why it is here.
        const double t_start = MonotonicSeconds();
        std::uint64_t ticks = 0;
        while (!g_shutdown.load() && !executor.stop_requested()
               && !task_error.load(std::memory_order_acquire)) {
            const auto st = robot.states();
            aico2_rt::BeginMeasuredWrite(*shm);
            aico2_rt::Measured& m = shm->measured;
            m.stamp_mono = MonotonicSeconds();
            for (std::uint32_t j = 0; j < dof; ++j) {
                m.q[j] = st.q[j];
                m.dq[j] = st.dq[j];
                m.tau[j] = st.tau[j];
            }
            m.dof = dof;
            m.operational = robot.operational() ? 1u : 0u;
            m.fault = robot.fault() ? 1u : 0u;
            aico2_rt::EndMeasuredWrite(*shm);

            if (++ticks % 200 == 0) {  // ~every 2 s
                std::printf("cycles %llu  missed %llu  max %.3f ms  state %u\n",
                    static_cast<unsigned long long>(executor.cycles()),
                    static_cast<unsigned long long>(executor.missed()),
                    executor.max_period() * 1e3,
                    static_cast<unsigned>(executor.state()));
            }
            if (opt.duration > 0.0 && MonotonicSeconds() - t_start >= opt.duration) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        scheduler.Stop();
        robot.Stop();

        if (task_error.load()) {
            std::fprintf(stderr, "\nthe control task threw: %s\n", err_text);
            rc = 1;
        } else if (executor.stop_requested()) {
            std::fprintf(stderr, "\nstopped: the robot reported a fault or left"
                                 " operational state\n");
            rc = 1;
        }

        std::printf("\n--- 1 kHz loop, achieved ---\n"
                    "cycles          %llu\n"
                    "mean period     %.6f ms  (nominal 1.000)\n"
                    "min period      %.6f ms\n"
                    "max period      %.6f ms\n"
                    "missed >1.5ms   %llu\n",
            static_cast<unsigned long long>(executor.cycles()), executor.mean_period() * 1e3,
            executor.min_period() * 1e3, executor.max_period() * 1e3,
            static_cast<unsigned long long>(executor.missed()));

    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        rc = 1;
    }

    munmap(shm, sizeof(aico2_rt::Shm));
    close(shm_fd);
    shm_unlink(opt.shm_name.c_str());
    return rc;
}
