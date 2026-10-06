/**
 * @file shm_protocol.hpp
 * @brief Shared-memory contract between the RT server and the ROS node.
 *
 * Two processes, because one process cannot currently hold both. A MoveIt
 * controller links rclcpp and therefore loads ROS 2's Fast-DDS; the only RDK
 * archive that works on this robot expects Flexiv's vendored Fast-DDS instead,
 * and the variant built to share ROS 2's copies stack-smashes against the
 * version Jazzy ships. See README.md. So:
 *
 *   aico2_rt_server   links the RDK, no ROS. Owns the robot session and the
 *                     1 kHz loop. Reads trajectories here, writes state here.
 *   aico2_rt_bridge   links rclcpp, no RDK. Owns the FollowJointTrajectory
 *                     action server and /joint_states. Writes trajectories
 *                     here, reads state here.
 *
 * Design rules, in order of importance:
 *
 *   1. The RT side never blocks. No mutexes, no waiting on the other process,
 *      no allocation. Everything is fixed-size and placed in the mapping.
 *   2. Either process may die at any moment without corrupting the other. No
 *      shared ownership, no handshake that can be left half-finished. A dead
 *      peer presents as a stale heartbeat, which both sides can act on.
 *   3. Plain data only. No pointers, no std::string, no virtuals -- the
 *      mapping lives at a different address in each process.
 *
 * Trajectories use three slots and an atomic index, which is what makes the
 * handoff lock-free in both directions: the writer picks a slot that is neither
 * the published one nor the one the reader says it is using, and with three
 * slots and at most two in use such a slot always exists. State uses a seqlock,
 * because the RT thread must publish without ever waiting for a reader.
 */

#ifndef AICO2_RT_CONTROL_SHM_PROTOCOL_HPP
#define AICO2_RT_CONTROL_SHM_PROTOCOL_HPP

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace aico2_rt {

/** Bump when the layout changes; both sides refuse a mismatch. */
constexpr std::uint32_t kShmVersion = 1;
constexpr std::uint32_t kShmMagic = 0x41494332;  // "AIC2"

/** 9 on this robot (2 external waist axes + 7 arm). Headroom costs nothing. */
constexpr std::size_t kMaxDof = 16;
/** A 30 s trajectory at MoveIt's density is a few hundred points. */
constexpr std::size_t kMaxPoints = 4096;
constexpr std::uint32_t kNumSlots = 3;
/** Published slot index meaning "nothing to execute". */
constexpr std::uint32_t kNoSlot = 0xFFFFFFFFu;

/** One MoveIt trajectory point. Matches trajectory_msgs/JointTrajectoryPoint
 *  minus `effort`, which MoveIt leaves empty. */
struct Point {
    double t;                 ///< time_from_start, seconds
    double q[kMaxDof];        ///< rad
    double dq[kMaxDof];       ///< rad/s
    double ddq[kMaxDof];      ///< rad/s^2
};

/** What the RT task is doing with the published trajectory. */
enum class ExecState : std::uint32_t {
    kIdle = 0,       ///< holding position, nothing adopted
    kRunning = 1,    ///< sampling a trajectory
    kFinished = 2,   ///< ran to the end, holding the last point
    kRejected = 3,   ///< validation failed; see reject_reason
    kAborted = 4,    ///< stopped early (fault, or cancel), holding
    kStopping = 5,   ///< decelerating to a stop after a cancel
};

/** The RT loop's nominal period, and the interval beyond which a cycle counts
 *  as having missed its deadline. */
constexpr double kLoopPeriodSec = 0.001;
constexpr double kDeadlineSec = 0.0015;

enum class RejectReason : std::uint32_t {
    kNone = 0,
    kStartPositionMismatch = 1,  ///< first point too far from measured q
    kNonZeroStartVelocity = 2,   ///< Flexiv require a zero-velocity start
    kNonMonotonicTime = 3,
    kTooManyPoints = 4,
    kDofMismatch = 5,
    kLimitExceeded = 6,          ///< outside the configured joint limits
    kNotOperational = 7,         ///< robot not enabled / in fault
    kNonZeroEndVelocity = 8,     ///< must end at rest, or holding it is a step
    kWaistMotionNotAllowed = 9,  ///< external axes commanded without --control-waist
    kUnknownJoint = 10,          ///< a goal joint name is not in the configured map
    kMissingJoint = 11,          ///< the goal omits a joint the server commands
    kServerNotRunning = 12,      ///< rt_server's heartbeat is stale
};

/** Human-readable, for action results and logs. Not used on the RT path. */
inline const char* RejectReasonName(RejectReason r)
{
    switch (r) {
        case RejectReason::kNone: return "none";
        case RejectReason::kStartPositionMismatch: return "start position does not match the robot";
        case RejectReason::kNonZeroStartVelocity: return "trajectory must start at rest";
        case RejectReason::kNonMonotonicTime: return "time_from_start must strictly increase";
        case RejectReason::kTooManyPoints: return "too many points";
        case RejectReason::kDofMismatch: return "joint count does not match the robot";
        case RejectReason::kLimitExceeded: return "outside joint limits";
        case RejectReason::kNotOperational: return "robot not operational";
        case RejectReason::kNonZeroEndVelocity: return "trajectory must end at rest";
        case RejectReason::kWaistMotionNotAllowed: return "waist motion requires --control-waist";
        case RejectReason::kUnknownJoint: return "goal names a joint the server does not know";
        case RejectReason::kMissingJoint: return "goal omits a joint the server commands";
        case RejectReason::kServerNotRunning: return "rt_server is not running";
    }
    return "unknown";
}

/** A trajectory, written by the bridge and read by the RT task. */
struct Slot {
    std::uint64_t id;          ///< goal id, echoed back in Status
    std::uint32_t n_points;
    std::uint32_t n_joints;
    Point points[kMaxPoints];
};

/**
 * @brief What the RT task publishes. Written only by the 1 kHz thread.
 *
 * Deliberately contains no measured joint data. `Robot::states()` is documented
 * as returning "a value copy of RobotStates struct", and RobotStates holds
 * twelve std::vector<double> -- so every call allocates, which is forbidden in
 * the control loop. Measured state is published separately by a non-RT thread,
 * where allocation is free. This is also why rt_hold_probe's timing result is
 * trustworthy: its periodic task called only fault(), never states().
 */
struct RtStatus {
    double stamp_mono;             ///< CLOCK_MONOTONIC seconds
    double cmd_q[kMaxDof];         ///< last commanded position
    double cmd_dq[kMaxDof];
    double cmd_ddq[kMaxDof];
    std::uint32_t dof;
    std::uint32_t exec_state;      ///< ExecState
    std::uint32_t reject_reason;   ///< RejectReason
    std::uint32_t clamped;         ///< a limit was hit at least once
    std::uint64_t active_id;       ///< goal currently adopted, 0 if none
    double traj_time;              ///< seconds into the active trajectory
    std::uint64_t cycles;
    std::uint64_t missed_deadlines;
    double max_period_sec;
    double min_period_sec;
};

/**
 * @brief Measured robot state. Written only by rt_server's non-RT thread.
 *
 * Separate from RtStatus so each has exactly one writer -- a seqlock with two
 * writers is not a seqlock. The bridge turns this into /joint_states.
 */
struct Measured {
    double stamp_mono;
    double q[kMaxDof];
    double dq[kMaxDof];
    double tau[kMaxDof];
    std::uint32_t dof;
    std::uint32_t operational;     ///< bool
    std::uint32_t fault;           ///< bool
    std::uint32_t _pad;
};

/**
 * @brief The mapping itself.
 *
 * Laid out so the two directions do not share a cache line: the RT task writes
 * `state_*` every cycle and reads `cmd_*` every cycle, and false sharing on a
 * 1 kHz loop is worth one padding array.
 */
struct Shm {
    std::uint32_t magic;
    std::uint32_t version;
    std::uint32_t dof;
    std::uint32_t _pad0;

    // ---- bridge -> RT ----------------------------------------------------
    /** Slot holding the newest trajectory, or kNoSlot. */
    std::atomic<std::uint32_t> published_slot;
    /** Incremented by the bridge on every publish. */
    std::atomic<std::uint64_t> publish_seq;
    /** Set by the bridge to ask the RT task to stop and hold. */
    std::atomic<std::uint32_t> cancel_request;
    /** Bridge heartbeat, so the RT task can hold position if the bridge dies. */
    std::atomic<std::uint64_t> bridge_heartbeat;

    alignas(64) char _pad1[64];

    // ---- RT -> bridge ----------------------------------------------------
    /** Slot the RT task is reading, so the bridge never overwrites it. */
    std::atomic<std::uint32_t> reading_slot;
    /** Echo of publish_seq once adopted; the bridge waits on this. */
    std::atomic<std::uint64_t> adopted_seq;
    /** Seqlock over `status`: even and unchanged across a read means coherent. */
    std::atomic<std::uint64_t> status_seq;
    /** RT heartbeat, so the bridge can tell a dead server from an idle one. */
    std::atomic<std::uint64_t> rt_heartbeat;
    RtStatus status;

    alignas(64) char _pad3[64];

    // ---- rt_server's non-RT thread -> bridge -----------------------------
    std::atomic<std::uint64_t> measured_seq;
    Measured measured;

    alignas(64) char _pad2[64];

    Slot slots[kNumSlots];
};

static_assert(std::atomic<std::uint32_t>::is_always_lock_free,
    "32-bit atomics must be lock-free to be usable across processes");
static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
    "64-bit atomics must be lock-free to be usable across processes");

constexpr const char* kDefaultShmName = "/aico2_rt_control";

// ---------------------------------------------------------------------------
// Writer side (bridge)
// ---------------------------------------------------------------------------

/**
 * @brief Pick a slot safe to overwrite.
 *
 * Neither the published slot nor the one the RT task reports reading. With
 * three slots and at most two in use, one is always free -- which is the whole
 * reason for the third.
 */
inline std::uint32_t PickFreeSlot(const Shm& shm)
{
    const std::uint32_t published = shm.published_slot.load(std::memory_order_acquire);
    const std::uint32_t reading = shm.reading_slot.load(std::memory_order_acquire);
    for (std::uint32_t i = 0; i < kNumSlots; ++i) {
        if (i != published && i != reading) {
            return i;
        }
    }
    return 0;  // unreachable while kNumSlots >= 3
}

/**
 * @brief Publish a slot already filled in.
 *
 * Release ordering, so the RT task cannot observe the new index before the
 * points it refers to.
 */
inline std::uint64_t PublishSlot(Shm& shm, std::uint32_t slot)
{
    const std::uint64_t seq = shm.publish_seq.load(std::memory_order_relaxed) + 1;
    shm.published_slot.store(slot, std::memory_order_release);
    shm.publish_seq.store(seq, std::memory_order_release);
    return seq;
}

// ---------------------------------------------------------------------------
// Reader side (RT task) -- must not block
// ---------------------------------------------------------------------------

/**
 * @brief Note which slot is being read, so the bridge leaves it alone.
 *
 * Call before touching the slot's points.
 */
inline void MarkReading(Shm& shm, std::uint32_t slot)
{
    shm.reading_slot.store(slot, std::memory_order_release);
}

/** @brief Acknowledge a publish; the bridge uses this to confirm adoption. */
inline void AcknowledgeSeq(Shm& shm, std::uint64_t seq)
{
    shm.adopted_seq.store(seq, std::memory_order_release);
}

/**
 * @brief Seqlock write, one pair per publisher. The writer never waits.
 *
 * Odd while writing, even when settled. A reader that sees an odd value, or two
 * different values either side of its read, retries.
 */
inline void BeginStatusWrite(Shm& shm)
{
    shm.status_seq.fetch_add(1, std::memory_order_release);
}
inline void EndStatusWrite(Shm& shm)
{
    shm.status_seq.fetch_add(1, std::memory_order_release);
}
inline void BeginMeasuredWrite(Shm& shm)
{
    shm.measured_seq.fetch_add(1, std::memory_order_release);
}
inline void EndMeasuredWrite(Shm& shm)
{
    shm.measured_seq.fetch_add(1, std::memory_order_release);
}

/**
 * @brief Read a coherent copy from a seqlock-protected region.
 * @return false if no stable copy was seen within `max_tries`.
 *
 * Bounded rather than a spin: the caller is the ROS side and would rather
 * publish nothing this cycle than block on a 1 kHz writer.
 */
template <typename T>
inline bool ReadSeqlocked(
    const std::atomic<std::uint64_t>& seq, const T& src, T& out, int max_tries = 8)
{
    for (int i = 0; i < max_tries; ++i) {
        const std::uint64_t before = seq.load(std::memory_order_acquire);
        if (before & 1u) {
            continue;  // a write is in progress
        }
        out = src;
        std::atomic_thread_fence(std::memory_order_acquire);
        if (seq.load(std::memory_order_acquire) == before) {
            return true;
        }
    }
    return false;
}

inline bool ReadStatus(const Shm& shm, RtStatus& out, int max_tries = 8)
{
    return ReadSeqlocked(shm.status_seq, shm.status, out, max_tries);
}
inline bool ReadMeasured(const Shm& shm, Measured& out, int max_tries = 8)
{
    return ReadSeqlocked(shm.measured_seq, shm.measured, out, max_tries);
}

}  // namespace aico2_rt

#endif  // AICO2_RT_CONTROL_SHM_PROTOCOL_HPP
