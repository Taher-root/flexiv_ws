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
 *
 * There are two ways to move the arm, and they are mutually exclusive:
 *
 *   trajectory  A whole time-parameterised plan is written into a slot and
 *               resampled at 1 kHz. This is MoveIt's Plan & Execute.
 *   servo       A single "latest target" is overwritten as fast as the producer
 *               likes, and a jerk-limited tracker chases it at 1 kHz. This is
 *               MoveIt Servo and VR teleop. Unlike a trajectory it carries no
 *               timing and never ends, so it is a different channel rather than
 *               a one-point trajectory: see servo_tracker.hpp.
 */

#ifndef AICO2_RT_CONTROL_SHM_PROTOCOL_HPP
#define AICO2_RT_CONTROL_SHM_PROTOCOL_HPP

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <time.h>

namespace aico2_rt {

/** Bump when the layout changes; both sides refuse a mismatch. */
constexpr std::uint32_t kShmVersion = 3;
constexpr std::uint32_t kShmMagic = 0x41494332;  // "AIC2"

/** 9 on this robot (2 external waist axes + 7 arm). Headroom costs nothing. */
constexpr std::size_t kMaxDof = 16;
/** A 30 s trajectory at MoveIt's density is a few hundred points. */
constexpr std::size_t kMaxPoints = 4096;
constexpr std::uint32_t kNumSlots = 3;
/** Published slot index meaning "nothing to execute". */
constexpr std::uint32_t kNoSlot = 0xFFFFFFFFu;

/** Per-joint bounds. The RT task clamps to these every cycle, and publishes
 *  them here so a writer can validate against exactly what will be enforced
 *  rather than against its own idea of the limits. */
struct Limits {
    double q_min[kMaxDof];
    double q_max[kMaxDof];
    double dq_max[kMaxDof];
    double ddq_max[kMaxDof];
};

/**
 * @brief How the tracker behaves. Written once by rt_server, read by everyone.
 *
 * Here rather than in rt_server's own memory so the bridge can report the
 * settings actually in force, and so a target writer can size its own rate
 * against them. A stale rt_server binary with different defaults is then
 * visible instead of guessed at.
 */
struct ServoConfig {
    /** Age at which a target stops being chased; see kDefaultServoTimeoutSec. */
    double timeout_sec;
    /** Step size treated as a glitch; see kDefaultServoMaxJumpRad. */
    double max_jump_rad;
    /** The terminal law's time constant: how the last fraction of a degree
     *  settles once the brake ceiling is no longer binding. NOT the lag -- the
     *  lag is set by ddq_max and max_jerk, and no value here changes it. See
     *  servo_tracker.hpp, which explains why the first-order "lookahead" this
     *  field used to be is unusable as a tracking law. */
    double settle_sec;
    /** Jerk limit, rad/s^3, applied per joint. The RDK takes no jerk limit of
     *  its own (docs/open_issues.md issue 1a), so this loop is the only place
     *  one exists. */
    double max_jerk[kMaxDof];
};

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
    kServoing = 6,   ///< tracking a streamed target, no trajectory
};

/** The RT loop's nominal period, and the interval beyond which a cycle counts
 *  as having missed its deadline. */
constexpr double kLoopPeriodSec = 0.001;
constexpr double kDeadlineSec = 0.0015;

/** Default age at which a servo target stops being chased. A producer running
 *  at 50-200 Hz refreshes every 5-20 ms, so 100 ms means five missed updates --
 *  late enough not to trip on scheduling noise, early enough that a dead
 *  producer brings the arm to a stop in about a tenth of a second. Overridable
 *  per run via Shm::servo_timeout_sec. */
constexpr double kDefaultServoTimeoutSec = 0.1;

/** Default distance a single servo target may sit from the current command
 *  before it is treated as a glitch and ignored. An IK solution that jumps
 *  branch, or a clutch that re-engages without re-seeding its offset, both
 *  arrive as a large step; chasing one means a fast unexpected move. */
constexpr double kDefaultServoMaxJumpRad = 0.5;

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
    kServoActive = 13,           ///< a trajectory arrived while servo mode was on
    kTrajectoryActive = 14,      ///< servo mode asked for while a trajectory runs
};

enum class ClampKind : std::uint32_t {
    kNone = 0,
    kPositionLow = 1,
    kPositionHigh = 2,
    kVelocity = 3,
    kAcceleration = 4,
};

inline const char* ClampKindName(ClampKind k)
{
    switch (k) {
        case ClampKind::kNone: return "none";
        case ClampKind::kPositionLow: return "position below q_min";
        case ClampKind::kPositionHigh: return "position above q_max";
        case ClampKind::kVelocity: return "velocity above dq_max";
        case ClampKind::kAcceleration: return "acceleration above ddq_max";
    }
    return "unknown";
}

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
        case RejectReason::kServoActive: return "servo mode is active; disable it first";
        case RejectReason::kTrajectoryActive: return "a trajectory is running";
    }
    return "unknown";
}

// ---------------------------------------------------------------------------
// Gripper. Separate from the RT loop: the gripper has its own device channel
// on the robot controller. Commands are written by the bridge and executed by
// rt_server's non-RT thread; state goes the other way.
// ---------------------------------------------------------------------------

enum class GripperCmd : std::uint32_t {
    kNone = 0,
    kGrasp = 1,   ///< force control: close with `force` N
    kMove = 2,    ///< position control: move to `width` m
    kStop = 3,    ///< hold current finger width
};

struct GripperCommand {
    std::uint32_t cmd;       ///< GripperCmd
    std::uint32_t _pad;
    double force;            ///< kGrasp: target force. kMove: force limit [N]
    double width;            ///< kMove: target opening [m]
    double velocity;         ///< kMove: finger velocity [m/s]
};

struct GripperState {
    double width;            ///< current finger opening [m]
    double force;            ///< current finger force [N]
    std::uint32_t is_moving; ///< fingers in motion
    std::uint32_t ready;     ///< gripper enabled and initialised
    double max_width;        ///< from GripperParams
    double max_force;        ///< from GripperParams
    double max_vel;          ///< from GripperParams
};

/** A trajectory, written by the bridge and read by the RT task. */
struct Slot {
    std::uint64_t id;          ///< goal id, echoed back in Status
    std::uint32_t n_points;
    std::uint32_t n_joints;
    Point points[kMaxPoints];
};

/**
 * @brief The latest streamed target. Written by the bridge, read every cycle.
 *
 * Deliberately NOT a one-point trajectory. A trajectory says "be at q at time
 * t"; this says "q is where I want you, as of when I wrote it". There is no
 * schedule, no end, and no guarantee another one is coming -- which is why the
 * RT side needs `stamp_mono` (to notice the producer stopped) and a tracker
 * rather than a sampler.
 *
 * Overwritten in place with no slot rotation: a target two updates old has no
 * value, so there is nothing to protect from being lost. The seqlock exists
 * only so the 1 kHz reader never sees half of one write mixed with half of the
 * next -- which on nine joints is a physically meaningless pose.
 */
struct ServoTarget {
    /** CLOCK_MONOTONIC seconds, taken by the writer. Compared against the RT
     *  loop's own clock, which is why both sides must use CLOCK_MONOTONIC. */
    double stamp_mono;
    double q[kMaxDof];         ///< target position, rad
    double dq[kMaxDof];        ///< target velocity, rad/s; feedforward only
    /** Bit j set means joint j is commanded. Unmasked joints hold where they
     *  are. MoveIt Servo on a 7-joint arm group sends seven joints, not the
     *  nine the robot vector has, so a mask is the honest representation --
     *  the alternative is the bridge guessing a hold position for the waist
     *  that only the RT loop actually knows. */
    std::uint32_t mask;
    std::uint32_t dof;
    /** Non-zero when `dq` came from the producer rather than being left empty.
     *  MoveIt Servo often omits velocities; a zero feedforward is correct in
     *  that case, but it is not the same as a producer that meant zero. */
    std::uint32_t have_dq;
    std::uint32_t _pad;
    std::uint64_t seq;         ///< increments on every write; 0 means never written
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
    /** Which joint and which bound clamped first. "YES, investigate" is not
     *  actionable on its own, and a validated trajectory should never clamp, so
     *  when it does the specific bound is the thing worth knowing. */
    std::uint32_t clamp_joint;
    std::uint32_t clamp_kind;      ///< ClampKind
    std::uint64_t active_id;       ///< goal currently adopted, 0 if none
    double traj_time;              ///< seconds into the active trajectory
    std::uint64_t cycles;
    std::uint64_t missed_deadlines;
    double max_period_sec;
    double min_period_sec;

    // ---- servo channel ---------------------------------------------------
    /** Non-zero while the newest servo target is older than servo_timeout_sec,
     *  i.e. the tracker is braking to a stop rather than following anything. */
    std::uint32_t servo_stale;
    std::uint32_t _pad_servo;
    /** Targets ignored for sitting further than servo_max_jump_rad from the
     *  current command. A non-zero count here is the signal that IK is jumping
     *  branch or the producer is not re-seeding its offset -- it would
     *  otherwise present only as the arm mysteriously not following. */
    std::uint64_t servo_rejected;
    /** Age of the target being tracked, seconds. The honest measure of how
     *  much of the end-to-end lag is upstream of this loop. */
    double servo_age_sec;
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
    /** Number of leading external axes; index < this is a waist axis. */
    std::uint32_t n_external;
    /** Non-zero when rt_server was started with --control-waist. */
    std::uint32_t control_waist;
    std::uint32_t _pad0;
    /** Written once by rt_server before the scheduler starts. */
    Limits limits;
    /** Likewise written once, before the scheduler starts. */
    ServoConfig servo_cfg;

    // ---- bridge -> RT ----------------------------------------------------
    /** Slot holding the newest trajectory, or kNoSlot. */
    std::atomic<std::uint32_t> published_slot;
    /** Incremented by the bridge on every publish. */
    std::atomic<std::uint64_t> publish_seq;
    /** Set by the bridge to ask the RT task to stop and hold. */
    std::atomic<std::uint32_t> cancel_request;
    /** Bridge heartbeat, so the RT task can hold position if the bridge dies. */
    std::atomic<std::uint64_t> bridge_heartbeat;
    /** Non-zero to put the loop in servo mode. The two channels are mutually
     *  exclusive: the executor refuses a trajectory while this is set, and the
     *  bridge refuses to set it while a trajectory runs. */
    std::atomic<std::uint32_t> servo_enable;

    alignas(64) char _pad1[64];

    // ---- bridge -> RT, servo target --------------------------------------
    // On its own cache line: written at up to a few hundred hertz by the
    // bridge while the RT loop reads it a thousand times a second, which is
    // exactly the pattern false sharing punishes.
    /** Seqlock over `servo_target`. One writer, the bridge. */
    std::atomic<std::uint64_t> servo_seq;
    ServoTarget servo_target;

    alignas(64) char _pad4[64];

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

    // ---- gripper: bridge -> rt_server (non-RT) --------------------------
    std::atomic<std::uint64_t> gripper_cmd_seq;
    GripperCommand gripper_cmd;

    // ---- gripper: rt_server (non-RT) -> bridge --------------------------
    std::atomic<std::uint64_t> gripper_state_seq;
    GripperState gripper_state;

    alignas(64) char _pad5[64];

    Slot slots[kNumSlots];
};

static_assert(std::atomic<std::uint32_t>::is_always_lock_free,
    "32-bit atomics must be lock-free to be usable across processes");
static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
    "64-bit atomics must be lock-free to be usable across processes");

constexpr const char* kDefaultShmName = "/aico2_rt_control";

/**
 * @brief The clock every `stamp_mono` in this mapping is taken from.
 *
 * Defined here, once, because it is part of the cross-process contract rather
 * than an implementation detail of either side. The servo target's age is the
 * difference between a stamp the bridge takes and a reading the RT loop takes,
 * so the two processes must be measuring the same thing -- and two reasonable
 * implementations are not interchangeable:
 *
 *   - rclcpp's now() is ROS time: wall clock by default, and possibly
 *     simulated. Minutes to decades away from CLOCK_MONOTONIC.
 *   - steady_clock::now() minus a t0 captured at process start, which is what
 *     rt_server used while it was the only process stamping anything. Correct
 *     within one process and meaningless across two, since each has its own
 *     t0. Against a bridge using absolute time it made every target look
 *     hours stale, so the tracker would have braked and held for ever.
 *
 * CLOCK_MONOTONIC, absolute, is the one definition both sides can arrive at
 * independently. Anything stamping into this mapping calls this function.
 */
inline double MonotonicSeconds()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + 1e-9 * static_cast<double>(ts.tv_nsec);
}

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
 *
 * The fences are the whole correctness argument and they are not symmetric, so
 * they are worth spelling out. Opening needs the counter to turn odd BEFORE any
 * data is written, which is a relaxed increment followed by a release fence --
 * the fence is what stops the data stores being hoisted above the increment.
 * Closing needs the reverse: all the data visible BEFORE the counter turns
 * even, which is a release fence followed by a relaxed increment.
 *
 * `fetch_add(release)` for both, which is the obvious thing to write and what
 * this did first, gets the opening backwards: a release increment orders
 * everything *before* it and says nothing about the data that follows, so the
 * compiler is free to sink the data stores past the increment and let a reader
 * observe an even counter either side of a copy that was in fact torn.
 *
 * No test has caught that happening -- the tests here measure zero torn reads
 * both before and after this change, on this compiler at -O2. It is a latent
 * bug fixed on the argument above, not an observed one, and it is written down
 * because the next person to simplify these four functions back into
 * `fetch_add(release)` will find nothing failing when they do.
 */
inline void BeginStatusWrite(Shm& shm)
{
    shm.status_seq.fetch_add(1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
}
inline void EndStatusWrite(Shm& shm)
{
    std::atomic_thread_fence(std::memory_order_release);
    shm.status_seq.fetch_add(1, std::memory_order_relaxed);
}
inline void BeginMeasuredWrite(Shm& shm)
{
    shm.measured_seq.fetch_add(1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
}
inline void EndMeasuredWrite(Shm& shm)
{
    std::atomic_thread_fence(std::memory_order_release);
    shm.measured_seq.fetch_add(1, std::memory_order_relaxed);
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

inline void BeginGripperCmdWrite(Shm& shm)
{
    shm.gripper_cmd_seq.fetch_add(1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
}
inline void EndGripperCmdWrite(Shm& shm)
{
    std::atomic_thread_fence(std::memory_order_release);
    shm.gripper_cmd_seq.fetch_add(1, std::memory_order_relaxed);
}
inline bool ReadGripperCmd(const Shm& shm, GripperCommand& out, int max_tries = 8)
{
    return ReadSeqlocked(shm.gripper_cmd_seq, shm.gripper_cmd, out, max_tries);
}

inline void BeginGripperStateWrite(Shm& shm)
{
    shm.gripper_state_seq.fetch_add(1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
}
inline void EndGripperStateWrite(Shm& shm)
{
    std::atomic_thread_fence(std::memory_order_release);
    shm.gripper_state_seq.fetch_add(1, std::memory_order_relaxed);
}
inline bool ReadGripperState(const Shm& shm, GripperState& out, int max_tries = 8)
{
    return ReadSeqlocked(shm.gripper_state_seq, shm.gripper_state, out, max_tries);
}

inline void BeginServoWrite(Shm& shm)
{
    shm.servo_seq.fetch_add(1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
}
inline void EndServoWrite(Shm& shm)
{
    std::atomic_thread_fence(std::memory_order_release);
    shm.servo_seq.fetch_add(1, std::memory_order_relaxed);
}

/**
 * @brief Read the newest servo target.
 * @return false if no coherent copy was seen, in which case `out` is unusable.
 *
 * Called from the 1 kHz task, so the retry budget is small on purpose: with a
 * reader at 1 kHz and a writer at a few hundred hertz a collision is rare and
 * one retry settles it, and a loop that cannot finish in four tries should
 * carry on with the target it already has rather than spend its cycle here.
 * Failing is cheap -- the tracker simply keeps chasing the previous target,
 * which is one update stale and still perfectly serviceable.
 */
inline bool ReadServoTarget(const Shm& shm, ServoTarget& out, int max_tries = 4)
{
    return ReadSeqlocked(shm.servo_seq, shm.servo_target, out, max_tries);
}

/**
 * @brief Overwrite the servo target. Writer side; not for the RT loop.
 * @param stamp_mono CLOCK_MONOTONIC seconds, the same clock the RT loop reads.
 */
inline void WriteServoTarget(Shm& shm, const double* q, const double* dq,
    std::uint32_t mask, std::uint32_t dof, bool have_dq, double stamp_mono)
{
    BeginServoWrite(shm);
    ServoTarget& t = shm.servo_target;
    t.stamp_mono = stamp_mono;
    for (std::uint32_t j = 0; j < kMaxDof; ++j) {
        t.q[j] = (j < dof) ? q[j] : 0.0;
        t.dq[j] = (j < dof && dq != nullptr) ? dq[j] : 0.0;
    }
    t.mask = mask;
    t.dof = dof;
    t.have_dq = have_dq ? 1u : 0u;
    t.seq = t.seq + 1;
    EndServoWrite(shm);
}

}  // namespace aico2_rt

#endif  // AICO2_RT_CONTROL_SHM_PROTOCOL_HPP
