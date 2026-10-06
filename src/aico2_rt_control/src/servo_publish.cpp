/**
 * @file servo_publish.cpp
 * @brief Drive the servo stream directly, with no ROS and no MoveIt.
 *
 * The servo twin of traj_publish, and it exists for the same reason: the RT
 * half of the servo path -- the shared-memory channel, the staleness timeout,
 * the jerk-limited tracker -- can be proven on the real arm before any of
 * MoveIt Servo, the teleop bridge or the headset is in the picture. If the arm
 * follows correctly here, every remaining problem is upstream of this loop.
 *
 * It is also the honest way to measure the lag. servo_tracker.hpp claims the
 * tracking lag is
 *
 *      v / (2 * ddq_max)  +  ddq_max / max_jerk    seconds
 *
 * and `--sine` sweeps a joint at a known speed so the claim can be checked
 * against the arm rather than against the unit test: the reported phase lag is
 * measured from the robot's own encoders.
 *
 * Deliberately awkward to fire by accident: nothing moves without --yes-move,
 * and the amplitude is capped unless the cap is explicitly raised.
 *
 *   ./servo_publish --list                                  # state only
 *   ./servo_publish --joint 6 --step 3 --yes-move            # hold 3 deg away
 *   ./servo_publish --joint 6 --sine 5 --period 4 --yes-move # sweep, report lag
 *   ./servo_publish --joint 6 --sine 5 --period 4 --drop 2 --yes-move
 *                                                           # stop feeding
 *                                                           # mid-sweep and
 *                                                           # watch it brake
 *
 * rt_server must already be running against the arm. Servo mode is enabled
 * here directly through shared memory, so rt_bridge need not be running --
 * which is the point.
 */

#include "aico2_rt_control/traj_ingest.hpp"

#include <chrono>
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
/** Amplitude cap, so a typo cannot ask for a large move. */
constexpr double kMaxDegreesDefault = 15.0;

struct Options {
    std::string shm_name = aico2_rt::kDefaultShmName;
    int joint = -1;
    double step_deg = 0.0;      ///< --step: hold this far from the start
    double sine_deg = 0.0;      ///< --sine: peak amplitude
    double period = 4.0;        ///< --period: sine period, seconds
    double seconds = 0.0;       ///< total run time; 0 = three periods, or 5 s
    double rate_hz = 100.0;     ///< producer rate, as MoveIt Servo would run
    double drop_after = 0.0;    ///< --drop: stop feeding after this many seconds
    double max_degrees = kMaxDegreesDefault;
    bool yes_move = false;
    bool list_only = false;
};

bool ParseArgs(int argc, char** argv, Options& o)
{
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const bool has_next = (i + 1 < argc);
        if (a == "--list") {
            o.list_only = true;
        } else if (a == "--yes-move") {
            o.yes_move = true;
        } else if (a == "--shm" && has_next) {
            o.shm_name = argv[++i];
        } else if (a == "--joint" && has_next) {
            o.joint = std::atoi(argv[++i]);
        } else if (a == "--step" && has_next) {
            o.step_deg = std::atof(argv[++i]);
        } else if (a == "--sine" && has_next) {
            o.sine_deg = std::atof(argv[++i]);
        } else if (a == "--period" && has_next) {
            o.period = std::atof(argv[++i]);
        } else if (a == "--seconds" && has_next) {
            o.seconds = std::atof(argv[++i]);
        } else if (a == "--rate" && has_next) {
            o.rate_hz = std::atof(argv[++i]);
        } else if (a == "--drop" && has_next) {
            o.drop_after = std::atof(argv[++i]);
        } else if (a == "--max-degrees" && has_next) {
            o.max_degrees = std::atof(argv[++i]);
        } else {
            std::printf(
                "usage: %s [options]\n"
                "  --list              print state and exit, moving nothing\n"
                "  --joint J           robot vector index (see --list)\n"
                "  --step D            hold D degrees from the start position\n"
                "  --sine A            sweep +/- A degrees instead\n"
                "  --period S          sine period in seconds (default 4)\n"
                "  --seconds S         run time (default: 3 periods, or 5 s)\n"
                "  --rate HZ           producer rate (default 100, as Servo)\n"
                "  --drop S            stop publishing after S seconds, to see\n"
                "                      the staleness timeout brake the arm\n"
                "  --max-degrees D     raise the %.0f degree cap deliberately\n"
                "  --shm NAME          shared memory name\n"
                "  --yes-move          actually move. Required.\n",
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

/** One sample of what the arm did, for the lag estimate afterwards. */
struct Sample {
    double t;        ///< seconds since the stream started
    double target;   ///< what was asked for, rad
    double measured; ///< where the arm was, rad
};

/**
 * @brief Phase lag of `measured` behind `target`, by cross-correlation.
 *
 * Shifting the measured series against the target and taking the shift with the
 * smallest squared error, rather than timing peaks: peak-finding on encoder
 * data is noisy, and the whole point is to compare against a formula to within
 * a few milliseconds. Only the second half of the run is used, so the initial
 * transient does not count as lag.
 */
double EstimateLagSeconds(const std::vector<Sample>& s, double dt)
{
    if (s.size() < 50) {
        return -1.0;
    }
    const std::size_t from = s.size() / 2;
    const std::size_t max_shift = s.size() / 4;
    double best = -1.0;
    double best_err = 0.0;
    for (std::size_t k = 0; k < max_shift; ++k) {
        double err = 0.0;
        std::size_t n = 0;
        for (std::size_t i = from; i + k < s.size(); ++i) {
            const double d = s[i].target - s[i + k].measured;
            err += d * d;
            ++n;
        }
        if (n == 0) {
            break;
        }
        err /= static_cast<double>(n);
        if (best < 0.0 || err < best_err) {
            best_err = err;
            best = static_cast<double>(k) * dt;
        }
    }
    return best;
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

    const ServoConfig& cfg = shm->servo_cfg;
    // The first arm axis, not index 0: without --control-waist the leading
    // external axes are pinned to a 1e-3 envelope, so their jerk limit is
    // 0.03 rad/s^3 and printing it reads as "no jerk limit configured".
    const std::uint32_t arm0 = shm->n_external < dof ? shm->n_external : 0u;
    std::printf("\nserver: dof %u, external axes %u, waist %s, exec state %u\n", dof,
        shm->n_external, shm->control_waist ? "COMMANDED" : "pinned", st.exec_state);
    std::printf("servo: arm jerk %.1f rad/s^3, timeout %.3f s, max jump %.3f rad,"
                " settle %.3f s\n",
        cfg.max_jerk[arm0], cfg.timeout_sec, cfg.max_jump_rad, cfg.settle_sec);
    std::printf("%-10s", "index");
    for (std::uint32_t j = 0; j < dof; ++j) {
        std::printf(" %8u", j);
    }
    std::printf("\n%-10s", "q (deg)");
    for (std::uint32_t j = 0; j < dof; ++j) {
        std::printf(" %8.2f", meas.q[j] * kRad2Deg);
    }
    std::printf("\n%-10s", "dq_max");
    for (std::uint32_t j = 0; j < dof; ++j) {
        std::printf(" %8.2f", shm->limits.dq_max[j]);
    }
    std::printf("\n%-10s", "ddq_max");
    for (std::uint32_t j = 0; j < dof; ++j) {
        std::printf(" %8.2f", shm->limits.ddq_max[j]);
    }
    std::printf("\n\n");

    if (opt.list_only) {
        return 0;
    }
    if (opt.joint < 0 || static_cast<std::uint32_t>(opt.joint) >= dof) {
        std::fprintf(stderr, "--joint must be 0..%u\n", dof - 1);
        return 1;
    }
    const auto j = static_cast<std::uint32_t>(opt.joint);
    if (!shm->control_waist && j < shm->n_external) {
        std::fprintf(stderr, "joint %u is an external (waist) axis and the server pinned it."
                             " Start rt_server with --control-waist to move it.\n", j);
        return 1;
    }
    const bool sine = std::fabs(opt.sine_deg) > 0.0;
    const double amp_deg = sine ? std::fabs(opt.sine_deg) : std::fabs(opt.step_deg);
    if (amp_deg <= 0.0) {
        std::fprintf(stderr, "give --step or --sine (or --list)\n");
        return 1;
    }
    if (amp_deg > opt.max_degrees) {
        std::fprintf(stderr, "refusing %.1f deg: over the %.1f deg cap. Raise --max-degrees"
                             " deliberately if that is really wanted.\n",
            amp_deg, opt.max_degrees);
        return 1;
    }
    if (opt.rate_hz < 5.0 || opt.rate_hz > 1000.0) {
        std::fprintf(stderr, "--rate must be 5..1000 Hz\n");
        return 1;
    }
    if (sine && opt.period <= 0.2) {
        std::fprintf(stderr, "--period must exceed 0.2 s\n");
        return 1;
    }

    const double q0 = meas.q[j];
    const double amp = amp_deg * kDeg2Rad * (sine ? 1.0 : (opt.step_deg < 0.0 ? -1.0 : 1.0));
    const double run = opt.seconds > 0.0 ? opt.seconds : (sine ? 3.0 * opt.period : 5.0);

    // The peak speed a sine of this amplitude and period demands, and the lag
    // the tracker predicts at that speed. Printed before moving, so the figure
    // to compare the measurement against is on screen either way.
    const double v_peak = sine ? (2.0 * M_PI * std::fabs(amp) / opt.period) : 0.0;
    const double a_max = shm->limits.ddq_max[j];
    const double j_max = cfg.max_jerk[j] > 0.0 ? cfg.max_jerk[j] : 30.0 * a_max;
    const double predicted = sine ? (v_peak / (2.0 * a_max) + a_max / j_max) : 0.0;

    std::printf("joint %u: %s about %.2f deg, %s, %.0f Hz producer, %.1f s\n", j,
        sine ? "sweeping +/-" : "stepping", amp_deg, sine ? "sine" : "hold", opt.rate_hz, run);
    if (sine) {
        std::printf("  peak speed %.3f rad/s; predicted lag %.0f ms"
                    " (%.0f from ddq_max %.1f, %.0f from max_jerk %.0f)\n",
            v_peak, 1000.0 * predicted, 1000.0 * v_peak / (2.0 * a_max), a_max,
            1000.0 * a_max / j_max, j_max);
    }
    if (opt.drop_after > 0.0) {
        std::printf("  will stop publishing after %.1f s; the arm should brake within"
                    " about %.0f ms of that\n", opt.drop_after, 1000.0 * cfg.timeout_sec);
    }
    if (std::fabs(amp) > cfg.max_jump_rad) {
        std::printf("  NOTE: the amplitude exceeds max_jump_rad (%.3f rad), so a step would"
                    " be refused as a glitch. A sine ramps there gradually and is fine.\n",
            cfg.max_jump_rad);
    }

    if (!opt.yes_move) {
        std::printf("\nNot published: --yes-move was not given. The arm WILL move with it.\n");
        return 0;
    }

    // Servo mode on. Done here rather than through rt_bridge on purpose: this
    // tool exists to test the RT half with nothing else running.
    shm->servo_enable.store(1u, std::memory_order_release);
    bool servoing = false;
    for (int i = 0; i < 500 && !servoing; ++i) {
        if (ReadStatus(*shm, st)
            && st.exec_state == static_cast<std::uint32_t>(ExecState::kServoing)) {
            servoing = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (!servoing) {
        std::fprintf(stderr, "the server did not enter servo mode (exec state %u)."
                             " Is a trajectory running?\n", st.exec_state);
        shm->servo_enable.store(0u, std::memory_order_release);
        return 1;
    }
    std::printf("\nservo mode on. Publishing...\n");

    const double dt = 1.0 / opt.rate_hz;
    const std::uint32_t mask = 1u << j;
    // Sampled at 1 kHz regardless of the producer rate. Recording only on
    // publish ticks caps the lag estimate's resolution at one producer period,
    // which at the default 100 Hz is 10 ms -- a fifth of the figure being
    // measured, and enough to make a correct tracker look wrong. The target at
    // a sample instant is known analytically, so there is no need to tie the
    // two rates together.
    constexpr double kSampleDt = 0.001;
    std::vector<Sample> samples;
    samples.reserve(static_cast<std::size_t>(run / kSampleDt) + 16);

    const auto target_at = [&](double t) {
        return sine ? (q0 + amp * std::sin(2.0 * M_PI * t / opt.period)) : (q0 + amp);
    };

    const double t_start = MonotonicSeconds();
    double next_pub = t_start;
    double next_sample = t_start;
    double q_target = q0;
    bool dropped = false;
    while (true) {
        const double now = MonotonicSeconds();
        const double t = now - t_start;
        if (t >= run) {
            break;
        }

        if (now >= next_pub) {
            next_pub += dt;
            if (opt.drop_after > 0.0 && t >= opt.drop_after) {
                if (!dropped) {
                    std::printf("  %.2f s: stopped publishing\n", t);
                    dropped = true;
                }
            } else {
                q_target = target_at(t);
                double q[kMaxDof] = {};
                q[j] = q_target;
                // No velocity feedforward: this is the case MoveIt Servo
                // actually produces, and the one the brake ceiling governs on
                // its own.
                WriteServoTarget(*shm, q, nullptr, mask, dof, false, now);
            }
        }

        if (now >= next_sample) {
            next_sample += kSampleDt;
            if (!dropped && ReadMeasured(*shm, meas)) {
                samples.push_back({t, target_at(t), meas.q[j]});
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    // Stop feeding and let the tracker brake, then leave servo mode.
    std::printf("  publishing stopped; waiting for the tracker to settle\n");
    shm->servo_enable.store(0u, std::memory_order_release);
    for (int i = 0; i < 2500; ++i) {
        if (ReadStatus(*shm, st)
            && st.exec_state != static_cast<std::uint32_t>(ExecState::kServoing)) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    ReadMeasured(*shm, meas);
    ReadStatus(*shm, st);
    std::printf("\nfinal exec state %u (0 = idle, 4 = aborted)\n", st.exec_state);
    std::printf("start %.3f deg, last target %.3f deg, measured %.3f deg\n", q0 * kRad2Deg,
        q_target * kRad2Deg, meas.q[j] * kRad2Deg);

    if (sine) {
        const double measured_lag = EstimateLagSeconds(samples, kSampleDt);
        if (measured_lag >= 0.0) {
            std::printf("measured lag %.0f ms over %zu samples, predicted %.0f ms\n",
                1000.0 * measured_lag, samples.size(), 1000.0 * predicted);
            // The measurement includes everything between the target being
            // written and /joint_states-equivalent data coming back, not just
            // the tracker: rt_server publishes measured state from a non-RT
            // thread, so its rate is a floor on what can be resolved here.
            std::printf("  (includes the measured-state publication delay, so it is an"
                        " upper bound on the tracker's own lag)\n");
        } else {
            std::printf("not enough samples to estimate the lag\n");
        }
    }

    std::printf("servo: %llu target(s) refused as glitches, stale %s, last target age %.1f ms\n",
        static_cast<unsigned long long>(st.servo_rejected), st.servo_stale ? "YES" : "no",
        1000.0 * st.servo_age_sec);
    if (st.clamped) {
        std::printf("CLAMPED: joint %u, %s\n", st.clamp_joint,
            ClampKindName(static_cast<ClampKind>(st.clamp_kind)));
    } else {
        std::printf("clamped: no\n");
    }
    std::printf("loop: cycles %llu, missed %llu, max period %.3f ms\n",
        static_cast<unsigned long long>(st.cycles),
        static_cast<unsigned long long>(st.missed_deadlines), st.max_period_sec * 1e3);

    munmap(shm, sizeof(Shm));
    close(fd);
    return 0;
}
