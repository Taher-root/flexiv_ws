/**
 * @file rt_hold_probe.cpp
 * @brief Does RT_JOINT_POSITION work on this arm, on this host, at 1 kHz?
 *
 * The cheapest question to answer before building anything larger. It holds
 * every joint at the position it started in and reports what the 1 kHz loop
 * actually achieved. No trajectory, no ROS, no motion beyond holding.
 *
 * Modelled on Flexiv's intermediate1_realtime_joint_position_control.cpp, with
 * three deliberate differences:
 *
 *   1. Nothing is allocated inside the periodic task. The Flexiv example
 *      constructs three std::vector<double> per cycle; that is fine for a demo
 *      and wrong in a control loop, because malloc can block on an unbounded
 *      lock. All buffers here are sized once before the scheduler starts.
 *   2. Nothing logs inside the periodic task. Logging takes a lock and formats
 *      strings. Statistics are accumulated in plain members written only by the
 *      RT thread and read only after Stop(), then printed from main.
 *   3. The achieved loop period is measured. That is the entire point: a 1 kHz
 *      RT loop on a kernel without PREEMPT_RT is best-effort, and in RT mode a
 *      missed deadline is worse than NRT because no internal motion generator
 *      is covering for you.
 *
 * Usage:
 *   rt_hold_probe Rizon4-063352                 # hold 5 s, report
 *   rt_hold_probe Rizon4-063352 --seconds 20
 *
 * Preconditions, same as every other RDK entry point here: E-stop released,
 * motion bar in Auto (Remote), no other process holding an RDK session to this
 * arm (the Python driver must be stopped -- one session per robot).
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>

#include <algorithm>
#include <atomic>
// Deliberately no spdlog, although Flexiv's examples log with it. The RDK
// headers need nothing but Eigen, and from v1.9.4.1 the library hides its
// internal symbols -- so including spdlog here drags in Ubuntu's build of it,
// which links fmt externally, and leaves this translation unit with undefined
// fmt::v9::* references unless fmt is found and linked too. Not worth a
// dependency for a few status lines; iostream covers it.
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr double kLoopPeriodSec = 0.001;
/** An interval beyond this counts as a missed deadline. 1.5x nominal. */
constexpr double kDeadlineSec = 0.0015;

std::atomic<bool> g_stop_sched{false};

/** Written only by the RT task, read only after Scheduler::Stop(). */
struct LoopStats {
    std::uint64_t cycles = 0;
    std::uint64_t misses = 0;
    double min_sec = 1e9;
    double max_sec = 0.0;
    double sum_sec = 0.0;
    bool have_prev = false;
    std::chrono::steady_clock::time_point prev{};
};

/** Preallocated so the periodic task never touches the allocator. */
struct Buffers {
    std::vector<double> target_pos;
    std::vector<double> target_vel;
    std::vector<double> target_acc;
};

void PrintHelp()
{
    std::cout << "Required: [robot_sn]   e.g. Rizon4-063352\n"
              << "Optional: --seconds N  hold duration, default 5\n"
              << "\n"
              << "Holds every joint where it is and reports the achieved 1 kHz\n"
              << "loop timing. The arm should not travel.\n";
}

/**
 * @brief The 1 kHz task. Allocation-free, lock-free, log-free.
 *
 * Holds target_pos at its initial value and streams it. Velocity and
 * acceleration stay zero: we are asking the controller to stand still, and a
 * nonzero feedforward would ask it to move.
 */
void PeriodicTask(flexiv::rdk::Robot& robot, Buffers& buf, LoopStats& stats)
{
    const auto now = std::chrono::steady_clock::now();
    if (stats.have_prev) {
        const double dt = std::chrono::duration<double>(now - stats.prev).count();
        stats.min_sec = std::min(stats.min_sec, dt);
        stats.max_sec = std::max(stats.max_sec, dt);
        stats.sum_sec += dt;
        if (dt > kDeadlineSec) {
            ++stats.misses;
        }
    }
    stats.prev = now;
    stats.have_prev = true;
    ++stats.cycles;

    // robot.fault() is a cached read, same as states(). Checking it here is
    // what the Flexiv example does and is the only way to notice a fault
    // without leaving the loop.
    if (robot.fault()) {
        g_stop_sched = true;
        return;
    }

    // Buffers already hold the hold-position target; nothing to recompute.
    robot.StreamJointPosition(buf.target_pos, buf.target_vel, buf.target_acc);
}

} // namespace

int main(int argc, char* argv[])
{
    if (argc < 2 || flexiv::rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }
    const std::string robot_sn = argv[1];

    double seconds = 5.0;
    for (int i = 2; i < argc - 1; ++i) {
        if (std::string(argv[i]) == "--seconds") {
            seconds = std::atof(argv[i + 1]);
        }
    }
    if (seconds <= 0.0 || seconds > 120.0) {
        std::cerr << "--seconds must be in (0, 120]\n";
        return 1;
    }

    try {
        flexiv::rdk::Robot robot(robot_sn);

        if (robot.fault()) {
            std::cout << "Fault present, clearing\n";
            if (!robot.ClearFault()) {
                std::cerr << "ClearFault failed\n";
                return 1;
            }
        }

        std::cout << "Enabling; E-stop must be released and motion bar in Auto (Remote)" << std::endl;
        robot.Enable();
        for (int i = 0; i < 30 && !robot.operational(); ++i) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        if (!robot.operational()) {
            std::cerr << "Not operational after 30s\n";
            return 1;
        }

        // This robot reports DoF 9: two external waist axes then seven arm
        // joints. StreamJointPosition takes the full vector, like
        // SendJointPosition does, so holding all nine is the safe probe. If RT
        // mode turns out to reject or ignore the external axes, that is one of
        // the things this program is here to find out.
        const auto dof = robot.info().DoF;
        std::cout << "DoF reported as " << dof << "\n";

        Buffers buf;
        buf.target_pos = robot.states().q;
        buf.target_pos.resize(dof);
        buf.target_vel.assign(dof, 0.0);
        buf.target_acc.assign(dof, 0.0);
        std::cout << "Holding at " << flexiv::rdk::utility::Vec2Str(buf.target_pos) << "\n";

        // The mode switch is where this either works or does not. If RT is not
        // available on this arm, expect it to throw here.
        std::cout << "Switching to RT_JOINT_POSITION" << std::endl;
        robot.SwitchMode(flexiv::rdk::Mode::RT_JOINT_POSITION);
        std::cout << "Mode switched; starting 1 kHz scheduler for " << seconds << "s" << std::endl;

        LoopStats stats;
        flexiv::rdk::Scheduler scheduler;
        scheduler.AddTask(std::bind(PeriodicTask, std::ref(robot), std::ref(buf), std::ref(stats)),
            "rt_hold", 1, scheduler.max_priority());
        scheduler.Start();

        const auto deadline = std::chrono::steady_clock::now()
                              + std::chrono::milliseconds(static_cast<long>(seconds * 1000));
        while (!g_stop_sched && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        scheduler.Stop();
        robot.Stop();

        // Read the stats only now that the RT thread has stopped writing them.
        const double mean_ms = stats.cycles > 1 ? (stats.sum_sec / (stats.cycles - 1)) * 1e3 : 0.0;
        std::cout << "\n--- 1 kHz loop, achieved ---\n"
                  << "cycles          " << stats.cycles << "\n"
                  << "expected        " << static_cast<std::uint64_t>(seconds / kLoopPeriodSec)
                  << "\n"
                  << "mean period     " << mean_ms << " ms  (nominal 1.000)\n"
                  << "min period      " << stats.min_sec * 1e3 << " ms\n"
                  << "max period      " << stats.max_sec * 1e3 << " ms\n"
                  << "missed >1.5ms   " << stats.misses << "\n";

        if (g_stop_sched) {
            std::cout << "\nStopped early: a fault appeared during the loop.\n";
            return 1;
        }
        if (stats.misses * 100 > stats.cycles) {
            std::cout << "\nOver 1% of cycles missed their deadline. On a kernel without\n"
                         "PREEMPT_RT this is expected under load, and it means RT control is\n"
                         "not safe to drive the arm with on this host as configured.\n";
        }
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
