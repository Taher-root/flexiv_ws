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

struct Options {
    std::string robot_sn;
    std::string shm_name = aico2_rt::kDefaultShmName;
    bool control_waist = false;
    double max_acc = 3.0;   // rad/s^2; matches the Python driver's constant
    /** Servo-stream settings. Zero means "take the documented default", which
     *  rt_executor substitutes; see kDefaultServoTimeoutSec and friends. The
     *  jerk limit defaults to a multiple of max_acc rather than a constant,
     *  because the tracker's lag carries a ddq_max / max_jerk term -- a fixed
     *  jerk limit would make raising --max-acc stop helping. */
    double servo_max_jerk = 0.0;
    double servo_timeout = 0.0;
    double servo_max_jump = 0.0;
    double servo_settle = 0.0;
    double duration = 0.0;  // 0 = until signalled
    /** 0 = stiff position control. >0 selects impedance at this fraction of
     *  the robot's nominal joint stiffness. */
    double stiffness_ratio = 0.0;
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
            "  --duration S        stop after S seconds (default: until Ctrl-C)\n"
            "  --impedance R       compliant instead of stiff: use RT_JOINT_IMPEDANCE\n"
            "                      with R * K_q_nom joint stiffness, R in (0, 1].\n"
            "                      R=1 is nominal stiffness, lower yields more.\n"
            "                      Arm axes only -- the external axes report an\n"
            "                      infinite K_q_nom and are not impedance\n"
            "                      controlled, so they stay rigid either way.\n"
            "\n"
            " servo stream (MoveIt Servo / VR teleop; see servo_tracker.hpp)\n"
            "  --servo-max-jerk J  jerk limit, rad/s^3 (default 30 * max-acc).\n"
            "                      The RDK has no jerk limit of its own, so\n"
            "                      this loop is the only place one exists.\n"
            "                      It is half the lag story: tracking lag is\n"
            "                      v/(2*max-acc) + max-acc/J seconds, so\n"
            "                      raising --max-acc without raising this\n"
            "                      eventually makes following WORSE.\n"
            "  --servo-timeout S   stop chasing a target older than S seconds\n"
            "                      (default %.2f). A dead producer brings the\n"
            "                      arm to a jerk-limited stop after this.\n"
            "  --servo-max-jump R  ignore a target more than R rad from the\n"
            "                      current command (default %.2f). Catches IK\n"
            "                      branch flips and un-seeded clutch\n"
            "                      re-engages, which would otherwise be\n"
            "                      followed at full speed.\n"
            "  --servo-settle T    terminal settling time constant, seconds\n"
            "                      (default 0.03). Shapes the last fraction\n"
            "                      of a degree only; it is NOT the lag.\n",
            argv[0], aico2_rt::kDefaultShmName,
            aico2_rt::kDefaultServoTimeoutSec, aico2_rt::kDefaultServoMaxJumpRad);
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
        } else if (a == "--servo-max-jerk" && has_next) {
            opt.servo_max_jerk = std::atof(argv[++i]);
        } else if (a == "--servo-timeout" && has_next) {
            opt.servo_timeout = std::atof(argv[++i]);
        } else if (a == "--servo-max-jump" && has_next) {
            opt.servo_max_jump = std::atof(argv[++i]);
        } else if (a == "--servo-settle" && has_next) {
            opt.servo_settle = std::atof(argv[++i]);
        } else if (a == "--impedance" && has_next) {
            opt.stiffness_ratio = std::atof(argv[++i]);
        } else {
            std::fprintf(stderr, "unrecognised argument: %s\n", a.c_str());
            return false;
        }
    }
    if (opt.servo_max_jerk < 0.0 || opt.servo_timeout < 0.0 || opt.servo_max_jump < 0.0
        || opt.servo_settle < 0.0) {
        std::fprintf(stderr, "the --servo-* values cannot be negative\n");
        return false;
    }
    if (opt.max_acc <= 0.0) {
        std::fprintf(stderr, "--max-acc must be positive\n");
        return false;
    }
    if (opt.stiffness_ratio < 0.0 || opt.stiffness_ratio > 1.0) {
        std::fprintf(stderr, "--impedance must be in (0, 1]: it is a fraction of the"
                             " robot's own nominal stiffness\n");
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
        // How wide the waist pin window is. Printed at startup, because a
        // stale binary with a narrower window refuses every goal and the
        // symptom ("outside joint limits") does not say which build is running.
        constexpr double kPinEpsilon = 1e-3;

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
            // A window, not exact equality, and it has to be wide enough for
            // two different things.
            //
            // Round-off: the sampler evaluates H0*q0 + H3*q1, where H0 + H3 is
            // 1 analytically but not in floating point, so pinning to a single
            // value leaves the result a ULP outside it and the clamp fires
            // every cycle.
            //
            // A live measurement: a writer builds its trajectory from the
            // CURRENT measured position, while this window is centred on the
            // position captured at startup. Those differ by however much the
            // arm has settled or the encoders have jittered since -- measured
            // tracking error alone is around 1.7e-5 rad -- so a window sized
            // for round-off rejects every goal with "outside joint limits".
            //
            // 1e-3 rad is 0.057 degrees: far too small to be motion anyone
            // would notice, and roughly 60 times the observed tracking error.
            for (std::uint32_t j = 0; j < ext && j < dof; ++j) {
                limits.q_min[j] = q_now[j] - kPinEpsilon;
                limits.q_max[j] = q_now[j] + kPinEpsilon;
                limits.dq_max[j] = kPinEpsilon;
                limits.ddq_max[j] = kPinEpsilon;
            }
            std::printf("waist NOT commanded: axes 0..%u pinned within +/-%.1e rad"
                        " (%.4f deg) of their current position\n",
                ext ? ext - 1 : 0, kPinEpsilon, kPinEpsilon * 57.29577951308232);
        } else {
            std::printf("waist WILL be commanded (--control-waist)\n");
        }

        // LockExternalAxes requires IDLE, so it must precede SwitchMode.
        if (ext > 0) {
            robot.LockExternalAxes(!opt.control_waist);
        }

        std::printf("holding at %s\n", flexiv::rdk::utility::Vec2Str(q_now).c_str());

        const bool impedance = opt.stiffness_ratio > 0.0;
        if (impedance) {
            std::printf("switching to RT_JOINT_IMPEDANCE (compliant)\n");
            robot.SwitchMode(flexiv::rdk::Mode::RT_JOINT_IMPEDANCE);
            // SetJointImpedance is only applicable in the impedance modes, so
            // it has to follow the switch, never precede it.
            //
            // The external axes report K_q_nom as infinite: they are not
            // impedance controlled, and the Python driver already established
            // that the right thing is to pass their nominal value straight
            // through and scale only the arm axes. Scaling infinity would be
            // meaningless, and substituting a finite number would be inventing
            // a stiffness the robot never offered.
            std::vector<double> K_q(info.K_q_nom);
            for (std::uint32_t j = ext; j < dof; ++j) {
                K_q[j] = info.K_q_nom[j] * opt.stiffness_ratio;
            }
            robot.SetJointImpedance(K_q);
            std::printf("joint stiffness set to %.0f%% of nominal on axes %u..%u:\n  %s\n",
                opt.stiffness_ratio * 100.0, ext, dof - 1,
                flexiv::rdk::utility::Vec2Str(K_q).c_str());
            std::printf("NOTE: a compliant arm does not track as closely by design."
                        " Position error is expected to grow as stiffness falls.\n");
        } else {
            std::printf("switching to RT_JOINT_POSITION (stiff)\n");
            robot.SwitchMode(flexiv::rdk::Mode::RT_JOINT_POSITION);
        }

        // Publish the configuration a writer needs in order to produce a
        // trajectory this server will accept without clamping. This has to
        // happen BEFORE Init: the executor reads n_external and servo_cfg out
        // of the mapping, so initialising first left it masking waist axes it
        // believed did not exist.
        shm->dof = dof;
        shm->n_external = ext;
        shm->control_waist = opt.control_waist ? 1u : 0u;
        shm->limits = limits;
        aico2_rt::ServoConfig scfg{};
        scfg.timeout_sec = opt.servo_timeout;
        scfg.max_jump_rad = opt.servo_max_jump;
        scfg.settle_sec = opt.servo_settle;
        for (std::uint32_t j = 0; j < aico2_rt::kMaxDof; ++j) {
            scfg.max_jerk[j] = opt.servo_max_jerk;
        }
        shm->servo_cfg = scfg;
        shm->servo_enable.store(0u);

        RobotAdapter adapter(robot, dof);
        aico2_rt::RtExecutor<RobotAdapter> executor;
        // Init substitutes the documented defaults for anything left at zero,
        // so the figures logged below are the ones actually in force.
        executor.Init(shm, &adapter, dof, limits, q_now.data());
        // Read back rather than recomputing: Init normalised the zeros into
        // the mapping, so these are the figures the loop will enforce. The
        // predicted lag is printed because it is the number anyone driving a
        // teleop stream actually wants, and it is not obvious from the limits.
        const aico2_rt::ServoConfig& eff = shm->servo_cfg;
        const double arm_acc = limits.ddq_max[ext < dof ? ext : 0];
        const double arm_jerk = eff.max_jerk[ext < dof ? ext : 0];
        std::printf("servo stream: jerk %.1f rad/s^3, timeout %.3f s, max jump %.3f rad,"
                    " settle %.3f s\n",
            arm_jerk, eff.timeout_sec, eff.max_jump_rad, eff.settle_sec);
        std::printf("  predicted tracking lag at 1 rad/s: %.0f ms"
                    " (%.0f from ddq_max, %.0f from max_jerk)\n",
            1000.0 * (1.0 / (2.0 * arm_acc) + arm_acc / arm_jerk),
            1000.0 / (2.0 * arm_acc), 1000.0 * arm_acc / arm_jerk);

        // Anything thrown out of the periodic task would cross the scheduler
        // boundary, so it is caught here and reported through a flag the main
        // thread polls. Nothing in the normal path throws.
        std::atomic<bool> task_error{false};
        static char err_text[256] = {};

        flexiv::rdk::Scheduler scheduler;
        scheduler.AddTask(
            [&] {
                try {
                    executor.Cycle(aico2_rt::MonotonicSeconds());
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
        const double t_start = aico2_rt::MonotonicSeconds();
        std::uint64_t ticks = 0;
        while (!g_shutdown.load() && !executor.stop_requested()
               && !task_error.load(std::memory_order_acquire)) {
            const auto st = robot.states();
            aico2_rt::BeginMeasuredWrite(*shm);
            aico2_rt::Measured& m = shm->measured;
            m.stamp_mono = aico2_rt::MonotonicSeconds();
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
            if (opt.duration > 0.0 && aico2_rt::MonotonicSeconds() - t_start >= opt.duration) {
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
