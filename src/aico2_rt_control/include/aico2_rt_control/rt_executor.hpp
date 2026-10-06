/**
 * @file rt_executor.hpp
 * @brief Everything the 1 kHz task does, with no dependency on the RDK.
 *
 * Separated from rt_server.cpp deliberately. The RDK is a prebuilt binary that
 * cannot be linked on a development machine, so logic left in the server would
 * only ever be exercised on the robot. Here it is templated on a minimal robot
 * interface, which means the state machine, the adoption handshake, the
 * clamping and the braking ramp are all unit-tested against a fake -- and
 * rt_server.cpp reduces to RDK calls.
 *
 * The robot type must provide, all as cheap cached reads:
 *
 *   bool fault() const;
 *   bool operational() const;
 *   void Stream(const double* q, const double* dq, const double* ddq);
 *
 * Note what is NOT in that list: any way to read measured joint state. The
 * RDK's `states()` returns a value copy of a struct holding twelve
 * std::vector<double>, so it allocates on every call and cannot appear in a
 * 1 kHz task. rt_server publishes measured state from a separate, non-RT
 * thread, which is also why rt_hold_probe's timing figures are trustworthy --
 * its periodic task called only fault().
 *
 * There are two ways the arm can be driven, and they are mutually exclusive
 * because both end at the same single stream of setpoints:
 *
 *   trajectory  A plan with timing, resampled by TrajSampler. MoveIt's Plan &
 *               Execute. Starts, ends, and is validated once up front.
 *   servo       A latest-target stream with no timing, chased by ServoTracker.
 *               MoveIt Servo and VR teleop. Never ends, and has to cope with
 *               the producer simply stopping -- which is why it needs a
 *               staleness timeout and the trajectory path does not.
 *
 * Switching between them only happens from a standstill. A trajectory arriving
 * mid-servo is refused, and servo mode is not entered while a trajectory runs;
 * leaving servo mode brakes to rest first rather than handing a live velocity
 * over to the hold path, which would be a step in the stream.
 *
 * RT rules, enforced by construction: nothing here allocates, takes a lock,
 * logs, or throws. Every buffer is a fixed-size member.
 */

#ifndef AICO2_RT_CONTROL_RT_EXECUTOR_HPP
#define AICO2_RT_CONTROL_RT_EXECUTOR_HPP

#include "aico2_rt_control/servo_tracker.hpp"
#include "aico2_rt_control/shm_protocol.hpp"
#include "aico2_rt_control/traj_sampler.hpp"

#include <cmath>
#include <cstdint>

namespace aico2_rt {

template <typename Robot>
class RtExecutor {
public:
    /**
     * @brief Prepare before the scheduler starts. Not RT-safe; call once.
     * @param hold_q Position captured on entering RT mode. Every path that has
     *               nothing valid to execute streams this, so it must be where
     *               the robot actually is.
     */
    void Init(Shm* shm, Robot* robot, std::uint32_t dof, const Limits& limits,
        const double* hold_q)
    {
        shm_ = shm;
        robot_ = robot;
        dof_ = dof;
        limits_ = limits;
        n_external_ = shm->n_external;
        for (std::uint32_t j = 0; j < kMaxDof; ++j) {
            cmd_.q[j] = (j < dof) ? hold_q[j] : 0.0;
            cmd_.dq[j] = 0.0;
            cmd_.ddq[j] = 0.0;
        }
        // Normalise the servo settings here rather than trusting the mapping.
        // A zeroed ServoConfig -- an older writer, or a test that never filled
        // it in -- would otherwise mean a zero timeout, which reads as "every
        // target is already stale" and presents as servo mode silently doing
        // nothing. Substituting the documented defaults fails visibly instead.
        servo_cfg_ = shm->servo_cfg;
        if (!(servo_cfg_.timeout_sec > 0.0)) {
            servo_cfg_.timeout_sec = kDefaultServoTimeoutSec;
        }
        if (!(servo_cfg_.max_jump_rad > 0.0)) {
            servo_cfg_.max_jump_rad = kDefaultServoMaxJumpRad;
        }
        if (!(servo_cfg_.settle_sec > 0.0)) {
            servo_cfg_.settle_sec = 0.03;
        }
        for (std::uint32_t j = 0; j < kMaxDof; ++j) {
            if (!(servo_cfg_.max_jerk[j] > 0.0)) {
                // A multiple of the acceleration limit, not a constant: the
                // tracker's lag carries a ddq_max / max_jerk term, so a fixed
                // jerk limit would make raising the acceleration limit stop
                // helping. See servo_tracker.hpp.
                servo_cfg_.max_jerk[j] = 30.0 * limits_.ddq_max[j];
            }
        }
        // Write the normalised settings back, so what the mapping advertises is
        // what the loop will actually enforce. Leaving the zeros in place would
        // make every reader -- the bridge, a target writer, a person looking at
        // the log -- re-derive these defaults and get them wrong when they
        // change. Safe to write directly: the scheduler has not started, so
        // there is no concurrent reader yet.
        shm->servo_cfg = servo_cfg_;
        state_ = ExecState::kIdle;
    }

    /**
     * @brief One control cycle. Call from the 1 kHz task and nowhere else.
     * @param now Monotonic seconds. Passed in rather than read here so tests
     *            can drive time directly.
     */
    void Cycle(double now)
    {
        UpdateTiming(now);

        // A fault means the robot has stopped accepting commands; streaming
        // into it is pointless and the main thread needs to know. Checked
        // first, because nothing below is meaningful afterwards.
        if (robot_->fault() || !robot_->operational()) {
            state_ = ExecState::kAborted;
            stop_requested_ = true;
            PublishStatus(now);
            return;
        }

        ArbitrateServoMode();

        if (shm_->cancel_request.load(std::memory_order_acquire) != 0u) {
            if (state_ == ExecState::kServoing) {
                // A cancel refers to a goal, and a stream has none. Clearing it
                // without acting is deliberate: braking here would stop the arm
                // while servo mode was still enabled, so the next target would
                // start it again and the cancel would look like a glitch. The
                // way to stop a stream is to turn servo mode off, which brakes
                // properly -- see ArbitrateServoMode.
            } else if (state_ == ExecState::kRunning) {
                // Do not jump to the measured position and do not drop the
                // velocity to zero in one cycle: both are discontinuities, and
                // Flexiv are explicit that the robot does not smooth RT
                // commands. Decelerate instead.
                state_ = ExecState::kStopping;
            } else if (state_ != ExecState::kStopping) {
                state_ = ExecState::kIdle;
                sampler_.Reset(nullptr);
            }
            shm_->cancel_request.store(0u, std::memory_order_release);
        }

        MaybeAdoptNewTrajectory(now);

        switch (state_) {
            case ExecState::kRunning: {
                Setpoint sp;
                const double t = now - traj_start_;
                if (sampler_.Sample(t, sp)) {
                    cmd_ = sp;
                } else {
                    // Sample() wrote the final point with zero rates, which is
                    // exactly what should be streamed from here on.
                    cmd_ = sp;
                    state_ = ExecState::kFinished;
                }
                traj_time_ = t;
                break;
            }
            case ExecState::kServoing:
                ServoCycle(now);
                break;
            case ExecState::kStopping:
                Decelerate();
                break;
            case ExecState::kIdle:
            case ExecState::kFinished:
            case ExecState::kAborted:
            case ExecState::kRejected:
                // Hold whatever was last commanded, with zero rates. Holding is
                // an active command: the stream must stay continuous.
                for (std::uint32_t j = 0; j < dof_; ++j) {
                    cmd_.dq[j] = 0.0;
                    cmd_.ddq[j] = 0.0;
                }
                break;
        }

        Clamp();
        robot_->Stream(cmd_.q, cmd_.dq, cmd_.ddq);
        PublishStatus(now);
    }

    bool stop_requested() const { return stop_requested_; }
    std::uint64_t cycles() const { return cycles_; }
    std::uint64_t missed() const { return missed_; }
    double max_period() const { return max_period_; }
    double min_period() const { return min_period_; }
    double mean_period() const { return cycles_ > 1 ? sum_period_ / (cycles_ - 1) : 0.0; }
    ExecState state() const { return state_; }
    const Setpoint& command() const { return cmd_; }
    RejectReason reject_reason() const { return reject_; }
    bool servo_stale() const { return servo_stale_; }
    std::uint64_t servo_rejected() const { return servo_.rejected(); }

private:
    void UpdateTiming(double now)
    {
        if (have_prev_) {
            const double dt = now - prev_;
            // Kept for the tracker, which integrates and so wants the interval
            // that actually elapsed rather than the nominal one. It bounds the
            // value itself; see ServoTracker::Step.
            last_dt_ = dt;
            min_period_ = dt < min_period_ ? dt : min_period_;
            max_period_ = dt > max_period_ ? dt : max_period_;
            sum_period_ += dt;
            if (dt > kDeadlineSec) {
                ++missed_;
            }
        }
        prev_ = now;
        have_prev_ = true;
        ++cycles_;
    }

    /**
     * @brief Adopt a newly published trajectory, if there is one.
     *
     * Validation proper happens in the bridge, which can afford it. The checks
     * here are the cheap ones that protect this loop from a malformed slot,
     * since a bad n_points would index out of the array.
     */
    void MaybeAdoptNewTrajectory(double now)
    {
        const std::uint64_t seq = shm_->publish_seq.load(std::memory_order_acquire);
        if (seq == seen_seq_) {
            return;
        }
        seen_seq_ = seq;

        const std::uint32_t slot = shm_->published_slot.load(std::memory_order_acquire);
        if (slot >= kNumSlots) {
            state_ = ExecState::kIdle;
            sampler_.Reset(nullptr);
            AcknowledgeSeq(*shm_, seq);
            return;
        }
        // A plan cannot be run while a stream is in charge: both drive the
        // same setpoint, and adopting one here would abandon the other
        // mid-motion. The bridge refuses such a goal up front, so reaching
        // this is a race; recording the reason keeps it diagnosable rather
        // than presenting as a goal that vanished. Servo mode continues --
        // dropping the stream because someone else sent a goal would be the
        // worse failure.
        if (state_ == ExecState::kServoing) {
            reject_ = RejectReason::kServoActive;
            AcknowledgeSeq(*shm_, seq);
            return;
        }

        // Tell the bridge which slot is in use before reading it, so it cannot
        // choose this one for the next trajectory.
        MarkReading(*shm_, slot);
        const Slot& s = shm_->slots[slot];

        if (s.n_points == 0 || s.n_points > kMaxPoints || s.n_joints != dof_) {
            state_ = ExecState::kRejected;
            reject_ = (s.n_joints != dof_) ? RejectReason::kDofMismatch
                                           : RejectReason::kTooManyPoints;
            sampler_.Reset(nullptr);
            AcknowledgeSeq(*shm_, seq);
            return;
        }

        sampler_.Reset(&s);
        traj_start_ = now;
        traj_time_ = 0.0;
        active_id_ = s.id;
        reject_ = RejectReason::kNone;
        state_ = ExecState::kRunning;
        AcknowledgeSeq(*shm_, seq);
    }

    /**
     * @brief Enter or leave servo mode, from a standstill in both directions.
     *
     * Entering is refused while a trajectory is in motion: the bridge checks
     * this too, so the guard is for the race, and the arm keeps doing what it
     * was already asked to do. Leaving does not take effect immediately --
     * `servo_release_` makes the tracker brake first, and only when it reports
     * at rest does the state fall back to a hold. Handing a live velocity
     * straight to the hold path would put a step in the stream, which is the
     * one thing Flexiv are explicit the robot will not smooth over.
     */
    void ArbitrateServoMode()
    {
        const bool want = shm_->servo_enable.load(std::memory_order_acquire) != 0u;
        if (state_ == ExecState::kServoing) {
            servo_release_ = !want;
            return;
        }
        if (!want) {
            return;
        }
        if (state_ == ExecState::kRunning || state_ == ExecState::kStopping) {
            reject_ = RejectReason::kTrajectoryActive;
            return;
        }
        // Seed from the current command, not from a measured position: the
        // command is what the robot is already tracking, and it cannot be read
        // back here anyway (states() allocates).
        servo_.Init(dof_, limits_, servo_cfg_, cmd_.q, cmd_.dq);
        servo_seen_seq_ = 0;
        servo_release_ = false;
        servo_stale_ = false;
        servo_age_ = 0.0;
        reject_ = RejectReason::kNone;
        sampler_.Reset(nullptr);
        active_id_ = 0;
        state_ = ExecState::kServoing;
    }

    /**
     * @brief One cycle of chasing the streamed target.
     *
     * The target is read fresh every cycle because the producer may have
     * replaced it; a failed seqlock read is not an error, it just means the
     * previous target stands for another millisecond.
     */
    void ServoCycle(double now)
    {
        ServoTarget t{};
        if (ReadServoTarget(*shm_, t) && t.seq != 0u && t.dof == dof_) {
            servo_age_ = now - t.stamp_mono;
            if (t.seq != servo_seen_seq_) {
                servo_seen_seq_ = t.seq;
                // The waist is pinned unless rt_server was told otherwise, and
                // a producer streaming an arm group has no business moving it.
                // Masking here rather than rejecting the sample means an arm
                // target that happens to carry waist bits still drives the arm.
                if (shm_->control_waist == 0u) {
                    for (std::uint32_t j = 0; j < n_external_ && j < kMaxDof; ++j) {
                        t.mask &= ~(1u << j);
                    }
                }
                servo_.SetTarget(t);
            }
        } else if (!servo_.have_target()) {
            servo_age_ = 0.0;
        }

        servo_stale_ = servo_.have_target() && (servo_age_ > servo_cfg_.timeout_sec);
        const bool follow = !servo_release_ && !servo_stale_;
        servo_.Step(last_dt_, follow, cmd_);

        if (servo_release_ && servo_.at_rest()) {
            // Braked to a standstill with servo mode off: hold here. kIdle
            // rather than kFinished because nothing was completed.
            servo_release_ = false;
            state_ = ExecState::kIdle;
        }
    }

    /** Ramp the commanded velocity to zero at ddq_max, integrating position. */
    void Decelerate()
    {
        bool moving = false;
        for (std::uint32_t j = 0; j < dof_; ++j) {
            const double a = limits_.ddq_max[j] > 0.0 ? limits_.ddq_max[j] : 1.0;
            const double step = a * kLoopPeriodSec;
            double v = cmd_.dq[j];
            if (v > step) {
                v -= step;
            } else if (v < -step) {
                v += step;
            } else {
                v = 0.0;
            }
            cmd_.q[j] += v * kLoopPeriodSec;
            cmd_.dq[j] = v;
            cmd_.ddq[j] = (v == 0.0) ? 0.0 : (v > 0.0 ? -a : a);
            if (v != 0.0) {
                moving = true;
            }
        }
        if (!moving) {
            state_ = ExecState::kAborted;
            sampler_.Reset(nullptr);
        }
    }

    /**
     * @brief Last line of defence before the joints.
     *
     * In NRT the robot's generator bounded whatever it was sent. In RT there is
     * nothing between this buffer and the hardware, so clamp every cycle even
     * though the bridge validates: the two guard different failures, and this
     * one also covers a trajectory that was valid when accepted but is being
     * sampled after a limit change.
     */
    void Clamp()
    {
        for (std::uint32_t j = 0; j < dof_; ++j) {
            if (cmd_.q[j] < limits_.q_min[j]) {
                cmd_.q[j] = limits_.q_min[j];
                NoteClamp(j, ClampKind::kPositionLow);
            } else if (cmd_.q[j] > limits_.q_max[j]) {
                cmd_.q[j] = limits_.q_max[j];
                NoteClamp(j, ClampKind::kPositionHigh);
            }
            const double vmax = limits_.dq_max[j];
            if (cmd_.dq[j] > vmax) {
                cmd_.dq[j] = vmax;
                NoteClamp(j, ClampKind::kVelocity);
            } else if (cmd_.dq[j] < -vmax) {
                cmd_.dq[j] = -vmax;
                NoteClamp(j, ClampKind::kVelocity);
            }
            const double amax = limits_.ddq_max[j];
            if (cmd_.ddq[j] > amax) {
                cmd_.ddq[j] = amax;
                NoteClamp(j, ClampKind::kAcceleration);
            } else if (cmd_.ddq[j] < -amax) {
                cmd_.ddq[j] = -amax;
                NoteClamp(j, ClampKind::kAcceleration);
            }
        }
    }

    /** Record only the first clamp: it is the one with a cause, and later ones
     *  are usually consequences of it. */
    void NoteClamp(std::uint32_t joint, ClampKind kind)
    {
        if (!clamped_) {
            clamped_ = true;
            clamp_joint_ = joint;
            clamp_kind_ = kind;
        }
    }

    /** Publish what this thread knows: the command and the loop's health. */
    void PublishStatus(double now)
    {
        BeginStatusWrite(*shm_);
        RtStatus& st = shm_->status;
        st.stamp_mono = now;
        for (std::uint32_t j = 0; j < dof_; ++j) {
            st.cmd_q[j] = cmd_.q[j];
            st.cmd_dq[j] = cmd_.dq[j];
            st.cmd_ddq[j] = cmd_.ddq[j];
        }
        st.dof = dof_;
        st.exec_state = static_cast<std::uint32_t>(state_);
        st.reject_reason = static_cast<std::uint32_t>(reject_);
        st.clamped = clamped_ ? 1u : 0u;
        st.clamp_joint = clamp_joint_;
        st.clamp_kind = static_cast<std::uint32_t>(clamp_kind_);
        st.active_id = active_id_;
        st.traj_time = traj_time_;
        st.cycles = cycles_;
        st.missed_deadlines = missed_;
        st.max_period_sec = max_period_;
        st.min_period_sec = min_period_;
        st.servo_stale = servo_stale_ ? 1u : 0u;
        st.servo_rejected = servo_.rejected();
        st.servo_age_sec = servo_age_;
        EndStatusWrite(*shm_);
        shm_->rt_heartbeat.store(cycles_, std::memory_order_release);
    }

    Shm* shm_ = nullptr;
    Robot* robot_ = nullptr;
    std::uint32_t dof_ = 0;
    Limits limits_{};
    std::uint32_t n_external_ = 0;
    TrajSampler sampler_;
    ServoTracker servo_;
    ServoConfig servo_cfg_{};
    Setpoint cmd_{};

    ExecState state_ = ExecState::kIdle;
    RejectReason reject_ = RejectReason::kNone;
    std::uint64_t seen_seq_ = 0;
    std::uint64_t active_id_ = 0;
    double traj_start_ = 0.0;
    double traj_time_ = 0.0;
    bool stop_requested_ = false;
    /** Servo mode was switched off; brake before dropping to a hold. */
    bool servo_release_ = false;
    bool servo_stale_ = false;
    double servo_age_ = 0.0;
    std::uint64_t servo_seen_seq_ = 0;
    bool clamped_ = false;
    std::uint32_t clamp_joint_ = 0;
    ClampKind clamp_kind_ = ClampKind::kNone;

    bool have_prev_ = false;
    double prev_ = 0.0;
    double last_dt_ = kLoopPeriodSec;
    double min_period_ = 1e9;
    double max_period_ = 0.0;
    double sum_period_ = 0.0;
    std::uint64_t cycles_ = 0;
    std::uint64_t missed_ = 0;
};

}  // namespace aico2_rt

#endif  // AICO2_RT_CONTROL_RT_EXECUTOR_HPP
