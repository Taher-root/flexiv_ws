/**
 * @file traj_ingest.hpp
 * @brief Turn a MoveIt goal into a validated slot. No ROS, no RDK.
 *
 * Everything the bridge decides lives here so it can be tested without a ROS
 * installation, for the same reason rt_executor.hpp exists: the parts that
 * cannot be compiled off the robot should contain no decisions.
 *
 * This is where Flexiv's two RT constraints are enforced. They are
 * goal-acceptance checks by nature -- the 1 kHz task cannot fix a trajectory
 * that starts in the wrong place, and discovering it at 1 kHz means discovering
 * it while the arm moves:
 *
 *   1. The trajectory must start at the robot's current position with zero
 *      velocity. MoveIt plans from /joint_states, which is a sample up to one
 *      publish period old, so the first point will be close but not identical.
 *   2. The stream must be continuous, because the robot does not smooth RT
 *      commands. A trajectory that ends with non-zero velocity cannot be held
 *      at its final point without a step, so it is rejected too.
 */

#ifndef AICO2_RT_CONTROL_TRAJ_INGEST_HPP
#define AICO2_RT_CONTROL_TRAJ_INGEST_HPP

#include "aico2_rt_control/rt_executor.hpp"
#include "aico2_rt_control/shm_protocol.hpp"

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace aico2_rt {

/** What the bridge is configured to accept. */
struct IngestConfig {
    std::uint32_t dof = 0;
    /** Number of leading external axes. Index < this is a waist axis. */
    std::uint32_t n_external = 0;
    /** True when rt_server was started with --control-waist. */
    bool allow_waist_motion = false;
    /** How far the first point may sit from the measured position, per joint. */
    double start_tolerance = 0.05;       // rad, ~2.9 deg
    /** How much velocity the first and last points may carry. */
    double rest_tolerance = 0.01;        // rad/s
    Limits limits{};
};

/**
 * @brief Map the goal's joint order onto the robot's vector order.
 *
 * MoveIt sends whatever order its planning group uses; the RDK vector is
 * external axes first. Done here, once per goal, rather than anywhere near the
 * control loop.
 *
 * @param goal_names  joint names in the order the goal lists them
 * @param rdk_names   joint names in robot vector order, length dof
 * @param out_map     out_map[i] is the robot index of goal joint i, or -1
 * @return kNone, or why the mapping is unusable
 */
inline RejectReason BuildJointMap(const std::vector<std::string>& goal_names,
    const std::vector<std::string>& rdk_names, std::vector<int>& out_map)
{
    out_map.assign(goal_names.size(), -1);
    for (std::size_t i = 0; i < goal_names.size(); ++i) {
        for (std::size_t j = 0; j < rdk_names.size(); ++j) {
            if (goal_names[i] == rdk_names[j]) {
                out_map[i] = static_cast<int>(j);
                break;
            }
        }
        if (out_map[i] < 0) {
            return RejectReason::kUnknownJoint;
        }
    }
    // Every robot joint must be covered: a partial goal would leave the rest
    // of the vector at whatever the previous trajectory left behind, which is
    // not something to guess at.
    std::vector<bool> seen(rdk_names.size(), false);
    for (const int m : out_map) {
        seen[static_cast<std::size_t>(m)] = true;
    }
    for (const bool s : seen) {
        if (!s) {
            return RejectReason::kMissingJoint;
        }
    }
    return RejectReason::kNone;
}

/**
 * @brief Check a trajectory already in robot index order.
 * @param q_meas Measured position, length dof.
 */
inline RejectReason ValidateTrajectory(
    const Point* pts, std::uint32_t n, const double* q_meas, const IngestConfig& cfg)
{
    if (n == 0 || n > kMaxPoints) {
        return RejectReason::kTooManyPoints;
    }
    if (pts[0].t < 0.0) {
        return RejectReason::kNonMonotonicTime;
    }
    for (std::uint32_t i = 1; i < n; ++i) {
        if (!(pts[i].t > pts[i - 1].t)) {
            return RejectReason::kNonMonotonicTime;
        }
    }
    for (std::uint32_t j = 0; j < cfg.dof; ++j) {
        if (std::fabs(pts[0].q[j] - q_meas[j]) > cfg.start_tolerance) {
            return RejectReason::kStartPositionMismatch;
        }
        if (std::fabs(pts[0].dq[j]) > cfg.rest_tolerance) {
            return RejectReason::kNonZeroStartVelocity;
        }
        if (std::fabs(pts[n - 1].dq[j]) > cfg.rest_tolerance) {
            return RejectReason::kNonZeroEndVelocity;
        }
    }
    // Waist motion when the server is not commanding the waist: the executor
    // would pin those axes and silently execute something other than the plan,
    // so say no instead.
    if (!cfg.allow_waist_motion) {
        for (std::uint32_t i = 0; i < n; ++i) {
            for (std::uint32_t j = 0; j < cfg.n_external && j < cfg.dof; ++j) {
                if (std::fabs(pts[i].q[j] - q_meas[j]) > cfg.start_tolerance) {
                    return RejectReason::kWaistMotionNotAllowed;
                }
            }
        }
    }
    for (std::uint32_t i = 0; i < n; ++i) {
        for (std::uint32_t j = 0; j < cfg.dof; ++j) {
            if (pts[i].q[j] < cfg.limits.q_min[j] || pts[i].q[j] > cfg.limits.q_max[j]
                || std::fabs(pts[i].dq[j]) > cfg.limits.dq_max[j]
                || std::fabs(pts[i].ddq[j]) > cfg.limits.ddq_max[j]) {
                return RejectReason::kLimitExceeded;
            }
        }
    }
    return RejectReason::kNone;
}

/** @brief Copy a validated trajectory into a slot. */
inline void FillSlot(
    Slot& slot, std::uint64_t id, const Point* pts, std::uint32_t n, std::uint32_t dof)
{
    slot.id = id;
    slot.n_points = n;
    slot.n_joints = dof;
    for (std::uint32_t i = 0; i < n; ++i) {
        slot.points[i] = pts[i];
    }
}

/** @brief Is rt_server alive? Its heartbeat is the RT cycle counter. */
inline bool ServerAlive(const Shm& shm, std::uint64_t& last_seen, int quiet_polls)
{
    const std::uint64_t hb = shm.rt_heartbeat.load(std::memory_order_acquire);
    const bool moved = (hb != last_seen);
    last_seen = hb;
    return moved || quiet_polls < 2;
}

}  // namespace aico2_rt

#endif  // AICO2_RT_CONTROL_TRAJ_INGEST_HPP
