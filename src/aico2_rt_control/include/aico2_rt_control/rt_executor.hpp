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
 *   const double* q() const;        // dof values
 *   const double* dq() const;
 *   const double* tau() const;
 *   void Stream(const double* q, const double* dq, const double* ddq);
 *
 * RT rules, enforced by construction: nothing here allocates, takes a lock,
 * logs, or throws. Every buffer is a fixed-size member.
 */

#ifndef AICO2_RT_CONTROL_RT_EXECUTOR_HPP
#define AICO2_RT_CONTROL_RT_EXECUTOR_HPP

#include "aico2_rt_control/shm_protocol.hpp"
#include "aico2_rt_control/traj_sampler.hpp"

#include <cmath>
#include <cstdint>

namespace aico2_rt {

/** Per-joint bounds. The RT task clamps to these every cycle. */
struct Limits {
    double q_min[kMaxDof];
    double q_max[kMaxDof];
    double dq_max[kMaxDof];
    double ddq_max[kMaxDof];
};

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
        for (std::uint32_t j = 0; j < kMaxDof; ++j) {
            cmd_.q[j] = (j < dof) ? hold_q[j] : 0.0;
            cmd_.dq[j] = 0.0;
            cmd_.ddq[j] = 0.0;
        }
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
            PublishState(now);
            return;
        }

        if (shm_->cancel_request.load(std::memory_order_acquire) != 0u) {
            if (state_ == ExecState::kRunning) {
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
        PublishState(now);
    }

    bool stop_requested() const { return stop_requested_; }
    std::uint64_t cycles() const { return cycles_; }
    std::uint64_t missed() const { return missed_; }
    double max_period() const { return max_period_; }
    double min_period() const { return min_period_; }
    double mean_period() const { return cycles_ > 1 ? sum_period_ / (cycles_ - 1) : 0.0; }
    ExecState state() const { return state_; }
    const Setpoint& command() const { return cmd_; }

private:
    void UpdateTiming(double now)
    {
        if (have_prev_) {
            const double dt = now - prev_;
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
                clamped_ = true;
            } else if (cmd_.q[j] > limits_.q_max[j]) {
                cmd_.q[j] = limits_.q_max[j];
                clamped_ = true;
            }
            const double vmax = limits_.dq_max[j];
            if (cmd_.dq[j] > vmax) {
                cmd_.dq[j] = vmax;
                clamped_ = true;
            } else if (cmd_.dq[j] < -vmax) {
                cmd_.dq[j] = -vmax;
                clamped_ = true;
            }
            const double amax = limits_.ddq_max[j];
            if (cmd_.ddq[j] > amax) {
                cmd_.ddq[j] = amax;
                clamped_ = true;
            } else if (cmd_.ddq[j] < -amax) {
                cmd_.ddq[j] = -amax;
                clamped_ = true;
            }
        }
    }

    void PublishState(double now)
    {
        BeginStateWrite(*shm_);
        State& st = shm_->state;
        st.stamp_mono = now;
        const double* q = robot_->q();
        const double* dq = robot_->dq();
        const double* tau = robot_->tau();
        for (std::uint32_t j = 0; j < dof_; ++j) {
            st.q[j] = q[j];
            st.dq[j] = dq[j];
            st.tau[j] = tau[j];
        }
        st.dof = dof_;
        st.operational = robot_->operational() ? 1u : 0u;
        st.fault = robot_->fault() ? 1u : 0u;
        st.exec_state = static_cast<std::uint32_t>(state_);
        st.reject_reason = static_cast<std::uint32_t>(reject_);
        st.active_id = active_id_;
        st.traj_time = traj_time_;
        st.cycles = cycles_;
        st.missed_deadlines = missed_;
        st.max_period_sec = max_period_;
        EndStateWrite(*shm_);
        shm_->rt_heartbeat.store(cycles_, std::memory_order_release);
    }

    Shm* shm_ = nullptr;
    Robot* robot_ = nullptr;
    std::uint32_t dof_ = 0;
    Limits limits_{};
    TrajSampler sampler_;
    Setpoint cmd_{};

    ExecState state_ = ExecState::kIdle;
    RejectReason reject_ = RejectReason::kNone;
    std::uint64_t seen_seq_ = 0;
    std::uint64_t active_id_ = 0;
    double traj_start_ = 0.0;
    double traj_time_ = 0.0;
    bool stop_requested_ = false;
    bool clamped_ = false;

    bool have_prev_ = false;
    double prev_ = 0.0;
    double min_period_ = 1e9;
    double max_period_ = 0.0;
    double sum_period_ = 0.0;
    std::uint64_t cycles_ = 0;
    std::uint64_t missed_ = 0;
};

}  // namespace aico2_rt

#endif  // AICO2_RT_CONTROL_RT_EXECUTOR_HPP
