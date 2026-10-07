/**
 * @file servo_tracker.hpp
 * @brief Chase a streamed target at 1 kHz under velocity, acceleration and
 *        jerk limits. Allocation-free, lock-free, no RDK and no ROS.
 *
 * The trajectory path resamples a plan that already carries timing, so the
 * sampler's job is to reproduce it faithfully (traj_sampler.hpp). The servo
 * path has no timing at all: a producer -- MoveIt Servo, or VR teleop --
 * overwrites "where I want the arm" whenever it likes, and this loop has to
 * turn that into a continuous 1 kHz stream the robot will accept. That is a
 * tracker, not an interpolator, and the difference matters: an interpolator is
 * told when to arrive, a tracker has to decide how fast to go.
 *
 * ---------------------------------------------------------------------------
 * The lag is physics, not a tuning parameter
 * ---------------------------------------------------------------------------
 *
 * The obvious design is a first-order lag: v = (q* - q) / L for some lookahead
 * L. It is smooth, cannot oscillate, and its lag is exactly L. It is also
 * unusable, and the reason is the whole character of this path.
 *
 * Following a target at speed v, a first-order law settles at a position error
 * of v * L. When the operator's hand stops, the arm has to kill speed v using
 * at most ddq_max, which takes v^2 / (2 * ddq_max) of travel. If the error
 * being carried is smaller than that, the arm physically cannot stop at the
 * target -- it overshoots. At ddq_max = 3 and v = 1 rad/s the stopping distance
 * is 0.17 rad, so a 50 ms lookahead carries 0.05 rad and overshoots by more
 * than 0.1: six degrees of arm still moving after the hand has stopped. Worse,
 * the law is infeasible at its own crossover for every L: equating it with the
 * brake law below puts the crossover at |e| = 2 * ddq_max * L^2, where
 * following it demands exactly 2 * ddq_max of deceleration. No value of L
 * fixes that, and smaller values make it worse.
 *
 * So the ceiling on speed is what can be braked out of -- the time-optimal law
 * for a bounded-acceleration system:
 *
 *      v <= sqrt(2 * ddq_max * |q* - q|)
 *
 * Tracking a ramp under that ceiling, plus the jerk term below, settles at
 *
 *      |e| ~= v^2 / (2 * ddq_max)  +  v * ddq_max / max_jerk
 *
 * and therefore a lag in seconds of
 *
 *      lag ~= v / (2 * ddq_max)  +  ddq_max / max_jerk
 *
 * Both terms matter, and they pull in opposite directions. The first falls as
 * the acceleration limit rises; the second RISES with it, because a larger
 * acceleration takes longer to turn around at a fixed jerk limit. Raising
 * ddq_max alone therefore stops paying off and eventually makes things worse.
 * The two improve together only if max_jerk rises with ddq_max, which is why
 * rt_server defaults max_jerk to a multiple of ddq_max rather than a constant.
 *
 * Worked, at 1 rad/s with max_jerk = 30 * ddq_max, so the second term is a
 * flat 33 ms:
 *
 *      ddq_max =  3  ->  167 + 33 = 200 ms
 *      ddq_max = 12  ->   42 + 33 =  75 ms
 *      ddq_max = 24  ->   21 + 33 =  54 ms
 *
 * There is no setting that removes it. Anything claiming otherwise is either
 * overshooting or exceeding its own acceleration limit. test_servo_tracker.cpp
 * asserts this formula numerically, so if it drifts, one of the two is wrong.
 *
 * ---------------------------------------------------------------------------
 * Jerk, and why the control input is jerk
 * ---------------------------------------------------------------------------
 *
 * The RDK applies no jerk limit of its own: its NRT generator switches between
 * +max_acc, 0 and -max_acc with no transition at all (docs/open_issues.md
 * issue 1a), which is one of the reasons for being here. So this is the only
 * place a jerk limit exists, and it shapes the whole design twice over.
 *
 * First, the brake ceiling has to pay for it. An acceleration that cannot step
 * must be turned around before it brakes, and over that interval the joint
 * keeps covering ground. The worst case is the one that actually happens --
 * accelerating at +ddq_max right up to the moment the ceiling bites, needing
 * the full 2 * ddq_max / max_jerk to reverse -- so the ceiling reserves that
 * travel before taking its square root. Reserving only the ramp up from zero
 * acceleration, which looks equally plausible, under-reserves by half: on a
 * 0.3 rad step at ddq_max = 3 that measured as 0.02 rad of overshoot followed
 * by a second of ringing, because every overshoot re-armed the same error in
 * the opposite direction.
 *
 * Second, the inner loop cannot be a deadbeat velocity controller. Asking for
 * the acceleration that would hit the velocity target within one cycle --
 * a = (v* - v) / dt -- is a rate-limited loop around a triple integrator, and
 * it limit-cycles: measured at 17 Hz and 0.037 rad/s of velocity ripple that
 * never decayed. So jerk is the control input, with explicit damping on both
 * velocity and acceleration:
 *
 *      jerk = (1/T^3) * e  +  (3/T^2) * (v* - v)  -  (3/T) * a
 *
 * the triple-pole critically damped law for a jerk-input plant. Those gains
 * make the terminal approach converge instead of ring, while the brake ceiling
 * above still governs whenever the loop is actually moving -- so the lag is
 * set by physics, and T only shapes the last fraction of a degree. T is small
 * on purpose: at 0.03 s the measured lag stays within 1% of the formula above,
 * where larger values start adding lag of their own.
 */

#ifndef AICO2_RT_CONTROL_SERVO_TRACKER_HPP
#define AICO2_RT_CONTROL_SERVO_TRACKER_HPP

#include "aico2_rt_control/shm_protocol.hpp"
#include "aico2_rt_control/traj_sampler.hpp"

#include <cmath>
#include <cstdint>

namespace aico2_rt {

/** Below these the joint is treated as stopped, and the command is snapped
 *  exactly to rest so that holding is genuinely static. The third-order law
 *  converges asymptotically and never reaches zero on its own, so without this
 *  a hold would creep and `at_rest()` would never become true. 1e-5 rad/s is
 *  0.0006 deg/s -- far below anything the arm can resolve. */
constexpr double kServoRestVel = 1e-5;
constexpr double kServoRestAcc = 1e-3;

/**
 * @brief Per-joint state for the chase. One instance, owned by the executor.
 *
 * Holds its own (q, v, a) rather than reading back the robot's measured state:
 * measured state cannot be read in the RT loop at all (`states()` allocates --
 * see rt_executor.hpp), and even if it could, closing a position loop around a
 * stream the robot is itself servoing would be two integrators fighting. The
 * command is the state, exactly as on the trajectory path.
 */
class ServoTracker {
public:
    /**
     * @brief Seed from the current command. Call on the transition into servo
     *        mode, which happens once.
     * @param q0 Where the arm is being held right now.
     * @param v0 The velocity currently commanded. Normally zero -- servo mode
     *           is entered from a hold -- but seeding from it rather than
     *           assuming zero keeps the command continuous if it ever is not.
     */
    void Init(std::uint32_t dof, const Limits& limits, const ServoConfig& cfg,
        const double* q0, const double* v0)
    {
        dof_ = dof;
        limits_ = limits;
        cfg_ = cfg;
        for (std::uint32_t j = 0; j < kMaxDof; ++j) {
            q_[j] = (j < dof && q0 != nullptr) ? q0[j] : 0.0;
            v_[j] = (j < dof && v0 != nullptr) ? v0[j] : 0.0;
            a_[j] = 0.0;
            tgt_q_[j] = q_[j];
            tgt_dq_[j] = 0.0;
        }
        mask_ = 0u;
        have_dq_ = false;
        have_target_ = false;
        rejected_ = 0;
    }

    /**
     * @brief Accept a new target, or refuse it as a glitch.
     * @return false if the target was ignored; the previous one stays in force.
     *
     * A target further than `max_jump_rad` from the current command is not
     * followed. A clutch that re-engages without re-seeding its offset, and an
     * IK solution that flips branch, both arrive looking exactly like this, and
     * both would otherwise produce a fast unexpected move -- the tracker would
     * do precisely what it was told, at full velocity, towards somewhere the
     * operator never pointed. Refusing is not silent: the count is published.
     *
     * Only masked joints are checked and only masked joints are stored, so a
     * seven-joint arm target cannot drag the waist with it.
     */
    bool SetTarget(const ServoTarget& t)
    {
        const std::uint32_t mask = t.mask;
        for (std::uint32_t j = 0; j < dof_ && j < kMaxDof; ++j) {
            if ((mask & (1u << j)) == 0u) {
                continue;
            }
            if (std::fabs(t.q[j] - q_[j]) > cfg_.max_jump_rad) {
                ++rejected_;
                return false;
            }
        }
        for (std::uint32_t j = 0; j < dof_ && j < kMaxDof; ++j) {
            if ((mask & (1u << j)) == 0u) {
                continue;
            }
            // Store the target already clamped into the joint's travel, so the
            // brake ceiling aims at the limit rather than through it. Clamping
            // the position afterwards instead -- the obvious place -- means the
            // tracker drives at the bound at whatever speed the error called
            // for and then has to kill that velocity in a single cycle, which
            // is a step in both velocity and acceleration. Aiming at the limit
            // brakes into it smoothly and leaves the position clamp in Step()
            // as a safety net that never has to fire.
            double q_t = t.q[j];
            if (q_t < limits_.q_min[j]) {
                q_t = limits_.q_min[j];
            } else if (q_t > limits_.q_max[j]) {
                q_t = limits_.q_max[j];
            }
            tgt_q_[j] = q_t;
            tgt_dq_[j] = t.dq[j];
        }
        mask_ = mask;
        have_dq_ = (t.have_dq != 0u);
        have_target_ = true;
        return true;
    }

    /**
     * @brief Advance one cycle and emit the command to stream.
     * @param dt     Elapsed seconds. Clamped to a sane window: integrating a
     *               scheduling hiccup verbatim would turn a 10 ms stall into a
     *               10 ms jump, which is the one thing the stream must never
     *               contain.
     * @param follow False to brake to a stop instead of chasing -- used when
     *               the target has gone stale, and when servo mode is being
     *               switched off. Braking is the same law with a zero error and
     *               a zero velocity goal, so the stop is jerk-limited exactly
     *               like everything else rather than being a second, less
     *               careful implementation.
     */
    void Step(double dt, bool follow, Setpoint& out)
    {
        if (!(dt > 0.5 * kLoopPeriodSec)) {
            dt = 0.5 * kLoopPeriodSec;
        } else if (dt > 2.0 * kLoopPeriodSec) {
            dt = 2.0 * kLoopPeriodSec;
        }
        const bool chase = follow && have_target_;

        const double T = cfg_.settle_sec > 0.0 ? cfg_.settle_sec : 0.03;
        const double k_pos = 1.0 / (T * T * T);
        const double k_vel = 3.0 / (T * T);
        const double k_acc = 3.0 / T;

        for (std::uint32_t j = 0; j < dof_ && j < kMaxDof; ++j) {
            const double v_max = limits_.dq_max[j] > 0.0 ? limits_.dq_max[j] : 0.0;
            const double a_max = limits_.ddq_max[j] > 0.0 ? limits_.ddq_max[j] : 0.0;
            const double j_max = cfg_.max_jerk[j] > 0.0 ? cfg_.max_jerk[j] : (30.0 * a_max);

            // Joints this target does not name hold position, and so does
            // everything when the producer has gone quiet. Both are expressed
            // as "no error, no target velocity", which the law below turns
            // into a jerk-limited stop wherever the joint happens to be.
            const bool masked = (mask_ & (1u << j)) != 0u;
            const bool track = chase && masked;
            const double e = track ? (tgt_q_[j] - q_[j]) : 0.0;
            const double v_ff = (track && have_dq_) ? tgt_dq_[j] : 0.0;

            // --- the brake ceiling, and the travel the reversal costs -------
            const double dir = (e >= 0.0) ? 1.0 : -1.0;
            const double v_dir = dir * v_[j];
            const double a_dir = dir * a_[j];
            double t_rev = (j_max > 0.0) ? ((a_dir + a_max) / j_max) : 0.0;
            if (t_rev < 0.0) {
                t_rev = 0.0;
            }
            // Exact when reversing from +ddq_max (the acceleration averages
            // zero over the reversal, so the distance is just v * t_rev) and
            // conservative from any lower acceleration -- the right way round
            // for a bound. Nothing is reserved when moving away from the
            // target, since that cannot overshoot it.
            const double e_rev = (v_dir > 0.0) ? (v_dir * t_rev) : 0.0;
            const double abs_e = std::fabs(e);
            const double e_eff = abs_e > e_rev ? abs_e - e_rev : 0.0;

            double v_ceil = std::sqrt(2.0 * a_max * e_eff);
            if (v_ceil > v_max) {
                v_ceil = v_max;
            }
            // The same ceiling against the joint's own travel limit, measured
            // along the direction of travel rather than of the error. The
            // target is already clamped into range so the error term respects
            // the limits by itself, but the feedforward is the producer's
            // velocity and at the limit the error is zero while the
            // feedforward may still point outwards. Covering it here makes
            // "cannot be commanded through a joint limit" a property of the
            // tracker rather than of the clamp that follows it.
            const double v_mag = std::fabs(v_[j]);
            const double e_rev_v = v_mag * t_rev;
            const double d_hi = limits_.q_max[j] - q_[j];
            const double d_lo = q_[j] - limits_.q_min[j];
            const double v_hi = std::sqrt(
                2.0 * a_max * (d_hi > e_rev_v ? d_hi - e_rev_v : 0.0));
            const double v_lo = std::sqrt(
                2.0 * a_max * (d_lo > e_rev_v ? d_lo - e_rev_v : 0.0));
            // Bound the feedforward by it, rather than leaving the override
            // below to catch it. At a limit the error term is zero while the
            // producer's velocity may still point outwards, and letting the
            // law ask for that speed and then vetoing it every cycle is a
            // bang-bang dither at exactly the place the arm is most likely to
            // be parked -- measured as 0.003 rad/s of buzz sitting on a stop.
            // Clipping the request instead leaves the law with nothing to
            // fight: at the limit the feedforward becomes zero, the remaining
            // terms are pure damping, and the joint simply rests.
            double v_goal = v_ff;
            if (v_goal > v_hi) {
                v_goal = v_hi;
            } else if (v_goal < -v_lo) {
                v_goal = -v_lo;
            }
            // The rest tolerance keeps the override itself from tripping on
            // the residual velocity of a converging hold.
            const double v_wall = (v_[j] >= 0.0 ? v_hi : v_lo) + kServoRestVel;

            // --- jerk command ----------------------------------------------
            double jerk;
            if (v_dir > v_ceil || v_mag > v_wall) {
                // Carrying more speed than can be stopped in the distance
                // left. Nothing else matters until that is fixed, so build the
                // brake at the maximum rate.
                jerk = -(v_[j] >= 0.0 ? 1.0 : -1.0) * j_max;
            } else {
                jerk = k_pos * e + k_vel * (v_goal - v_[j]) - k_acc * a_[j];
            }
            if (jerk > j_max) {
                jerk = j_max;
            } else if (jerk < -j_max) {
                jerk = -j_max;
            }

            // --- integrate, each stage bounded -----------------------------
            a_[j] += jerk * dt;
            if (a_[j] > a_max) {
                a_[j] = a_max;
            } else if (a_[j] < -a_max) {
                a_[j] = -a_max;
            }
            v_[j] += a_[j] * dt;
            if (v_[j] > v_max) {
                v_[j] = v_max;
            } else if (v_[j] < -v_max) {
                v_[j] = -v_max;
            }
            q_[j] += v_[j] * dt;

            // Hard position bound, and only the position. The brake ceilings
            // above hold the velocity to what can be stopped in the distance
            // left, but the jerk limit means the acceleration is still being
            // turned around as the joint arrives, so a sub-milliradian
            // overshoot is possible and this trims it.
            //
            // Zeroing the velocity and acceleration here as well -- the
            // obvious thing, and what this did first -- is wrong. It puts a
            // step in the streamed dq, which the robot takes as a feedforward
            // and turns into a torque spike, in exchange for fixing an
            // inconsistency of 1e-3 rad that nothing downstream reads. With
            // the position pinned the error term is zero, so the law below is
            // already braking at full damping and the velocity is gone within
            // a few cycles. A smooth, briefly inconsistent dq beats a
            // discontinuous one.
            if (q_[j] < limits_.q_min[j]) {
                q_[j] = limits_.q_min[j];
            } else if (q_[j] > limits_.q_max[j]) {
                q_[j] = limits_.q_max[j];
            }

            // Settle exactly, so that holding is static rather than creeping.
            if (!track && std::fabs(v_[j]) < kServoRestVel
                && std::fabs(a_[j]) < kServoRestAcc) {
                v_[j] = 0.0;
                a_[j] = 0.0;
            }

            out.q[j] = q_[j];
            out.dq[j] = v_[j];
            out.ddq[j] = a_[j];
        }
        for (std::uint32_t j = dof_; j < kMaxDof; ++j) {
            out.q[j] = 0.0;
            out.dq[j] = 0.0;
            out.ddq[j] = 0.0;
        }
    }

    /** True when nothing is moving. Used to decide a hold has settled, so the
     *  thresholds are the ones Step() snaps to rather than exact zero. */
    bool at_rest() const
    {
        for (std::uint32_t j = 0; j < dof_ && j < kMaxDof; ++j) {
            if (v_[j] != 0.0 || a_[j] != 0.0) {
                return false;
            }
        }
        return true;
    }

    std::uint64_t rejected() const { return rejected_; }
    bool have_target() const { return have_target_; }
    std::uint32_t mask() const { return mask_; }

private:
    std::uint32_t dof_ = 0;
    Limits limits_{};
    ServoConfig cfg_{};

    double q_[kMaxDof]{};
    double v_[kMaxDof]{};
    double a_[kMaxDof]{};
    double tgt_q_[kMaxDof]{};
    double tgt_dq_[kMaxDof]{};
    std::uint32_t mask_ = 0u;
    bool have_dq_ = false;
    bool have_target_ = false;
    std::uint64_t rejected_ = 0;
};

}  // namespace aico2_rt

#endif  // AICO2_RT_CONTROL_SERVO_TRACKER_HPP
