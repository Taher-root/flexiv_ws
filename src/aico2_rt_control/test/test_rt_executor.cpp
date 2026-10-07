// Exercises the 1 kHz state machine against a fake robot: adoption, sampling,
// clamping, the braking ramp, and fault handling. This is the code that will
// command real joints, and the RDK cannot be linked off the robot, so it is
// tested here instead.

#include "aico2_rt_control/rt_executor.hpp"

#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

using namespace aico2_rt;

namespace {

int g_failures = 0;

void Check(bool ok, const char* what)
{
    if (!ok) {
        ++g_failures;
        std::printf("  FAIL  %s\n", what);
    }
}
void Close(double got, double want, double tol, const char* what)
{
    if (!(std::fabs(got - want) <= tol)) {
        ++g_failures;
        std::printf("  FAIL  %s (got %.9g, want %.9g)\n", what, got, want);
    }
}

constexpr std::uint32_t kDof = 9;

/** Records what it was told to stream, and reports whatever state we set. */
struct FakeRobot {
    bool faulted = false;
    bool op = true;
    std::vector<Setpoint> streamed;

    bool fault() const { return faulted; }
    bool operational() const { return op; }
    void Stream(const double* q, const double* dq, const double* ddq)
    {
        Setpoint s{};
        for (std::uint32_t j = 0; j < kMaxDof; ++j) {
            s.q[j] = q[j];
            s.dq[j] = dq[j];
            s.ddq[j] = ddq[j];
        }
        streamed.push_back(s);
    }
};

Limits WideLimits()
{
    Limits l{};
    for (std::uint32_t j = 0; j < kMaxDof; ++j) {
        l.q_min[j] = -10.0;
        l.q_max[j] = 10.0;
        l.dq_max[j] = 10.0;
        l.ddq_max[j] = 20.0;
    }
    return l;
}

std::unique_ptr<Shm> FreshShm()
{
    auto shm = std::make_unique<Shm>();
    shm->magic = kShmMagic;
    shm->version = kShmVersion;
    shm->dof = kDof;
    shm->published_slot.store(kNoSlot);
    shm->reading_slot.store(kNoSlot);
    shm->servo_cfg.timeout_sec = 0.1;
    shm->servo_cfg.max_jump_rad = 0.5;
    shm->servo_cfg.settle_sec = 0.03;
    for (std::uint32_t j = 0; j < kMaxDof; ++j) {
        shm->servo_cfg.max_jerk[j] = 400.0;   // 30 * ddq_max from WideLimits
    }
    return shm;
}

/** Write a servo target naming every joint, as a producer would. */
void PublishServoTarget(Shm& shm, const double* q, double stamp_mono,
    std::uint32_t mask = (1u << kDof) - 1u)
{
    BeginServoWrite(shm);
    ServoTarget& t = shm.servo_target;
    t.stamp_mono = stamp_mono;
    for (std::uint32_t j = 0; j < kMaxDof; ++j) {
        t.q[j] = (j < kDof) ? q[j] : 0.0;
        t.dq[j] = 0.0;
    }
    t.mask = mask;
    t.dof = kDof;
    t.have_dq = 0u;
    t.seq = t.seq + 1;
    EndServoWrite(shm);
}

/** Fill a slot with a ramp on joint 0 from `from` to `to` over `T` seconds,
 *  at rest at both ends, as MoveIt's output is. */
void FillRamp(Slot& s, std::uint64_t id, double from, double to, double T, int n)
{
    s.id = id;
    s.n_points = static_cast<std::uint32_t>(n);
    s.n_joints = kDof;
    for (int i = 0; i < n; ++i) {
        const double u = static_cast<double>(i) / (n - 1);
        // Quintic ease, so dq and ddq vanish at both ends.
        const double e = 10 * u * u * u - 15 * u * u * u * u + 6 * u * u * u * u * u;
        const double de = (30 * u * u - 60 * u * u * u + 30 * u * u * u * u) / T;
        const double dde = (60 * u - 180 * u * u + 120 * u * u * u) / (T * T);
        Point& p = s.points[i];
        p.t = T * u;
        for (std::uint32_t j = 0; j < kDof; ++j) {
            p.q[j] = (j == 0) ? from + (to - from) * e : 0.0;
            p.dq[j] = (j == 0) ? (to - from) * de : 0.0;
            p.ddq[j] = (j == 0) ? (to - from) * dde : 0.0;
        }
    }
}

/** Servo mode is entered from a hold, tracks a streamed target, and is left
 *  by braking rather than by dropping a live velocity on the floor. */
void TestServoModeTracksAStreamedTarget()
{
    std::printf("servo mode enters from a hold, tracks, and brakes on exit\n");
    auto shm = FreshShm();
    FakeRobot robot;
    RtExecutor<FakeRobot> ex;
    double hold[kMaxDof] = {};
    ex.Init(shm.get(), &robot, kDof, WideLimits(), hold);

    double now = 0.0;
    ex.Cycle(now);
    Check(ex.state() == ExecState::kIdle, "starts idle");

    shm->servo_enable.store(1u);
    now += kLoopPeriodSec;
    ex.Cycle(now);
    Check(ex.state() == ExecState::kServoing, "enters servo mode from idle");

    // Stream a target 0.2 rad away on joint 4, refreshed every cycle so it
    // never goes stale, and let the tracker converge.
    double tgt[kMaxDof] = {};
    tgt[4] = 0.2;
    for (int i = 0; i < 3000; ++i) {
        now += kLoopPeriodSec;
        PublishServoTarget(*shm, tgt, now);
        ex.Cycle(now);
    }
    Check(ex.state() == ExecState::kServoing, "stays in servo mode while fed");
    Check(!ex.servo_stale(), "a target refreshed every cycle is never stale");
    Close(ex.command().q[4], 0.2, 2e-4, "converges on the streamed target");
    Close(ex.command().q[0], 0.0, 1e-12, "other joints untouched");

    // Now move the target so the arm is genuinely in motion, then switch servo
    // mode off mid-flight.
    tgt[4] = 0.6;
    for (int i = 0; i < 150; ++i) {
        now += kLoopPeriodSec;
        PublishServoTarget(*shm, tgt, now);
        ex.Cycle(now);
    }
    Check(std::fabs(ex.command().dq[4]) > 0.05, "was moving when servo was switched off");

    shm->servo_enable.store(0u);
    const std::size_t at_release = robot.streamed.size();
    for (int i = 0; i < 3000; ++i) {
        now += kLoopPeriodSec;
        ex.Cycle(now);
    }
    Check(ex.state() == ExecState::kIdle, "falls back to a hold once stopped");
    Close(ex.command().dq[4], 0.0, 1e-12, "stopped with exactly zero velocity");

    // The deceleration must be a ramp, not a step: the whole reason leaving
    // servo mode is deferred until the tracker reports at rest.
    double worst = 0.0;
    for (std::size_t i = at_release + 1; i < robot.streamed.size(); ++i) {
        worst = std::fmax(worst,
            std::fabs(robot.streamed[i].dq[4] - robot.streamed[i - 1].dq[4]));
    }
    Check(worst <= WideLimits().ddq_max[4] * kLoopPeriodSec + 1e-9,
        "release decelerates within ddq_max, with no velocity step");
}

/** A producer that stops being refreshed must bring the arm to a stop. */
void TestStaleServoTargetBrakes()
{
    std::printf("a stale servo target brakes the arm and says so\n");
    auto shm = FreshShm();
    FakeRobot robot;
    RtExecutor<FakeRobot> ex;
    double hold[kMaxDof] = {};
    ex.Init(shm.get(), &robot, kDof, WideLimits(), hold);
    shm->servo_enable.store(1u);

    double now = 0.0;
    double tgt[kMaxDof] = {};
    // Within max_jump_rad: a single target further than that is refused as a
    // glitch, which would leave nothing being tracked at all.
    tgt[4] = 0.4;
    for (int i = 0; i < 300; ++i) {   // come up to speed
        now += kLoopPeriodSec;
        PublishServoTarget(*shm, tgt, now);
        ex.Cycle(now);
    }
    Check(ex.command().dq[4] > 0.1, "moving before the producer stops");
    Check(!ex.servo_stale(), "not stale while being fed");

    // Stop refreshing. The target's stamp stays where it was, so its age grows
    // past servo_cfg.timeout_sec purely from the loop's own clock.
    for (int i = 0; i < 3000; ++i) {
        now += kLoopPeriodSec;
        ex.Cycle(now);
    }
    Check(ex.servo_stale(), "reports the target as stale");
    Check(ex.state() == ExecState::kServoing, "still in servo mode, just holding");
    Close(ex.command().dq[4], 0.0, 1e-12, "brakes to a standstill");

    // And picks straight back up when the producer returns, without needing
    // servo mode to be cycled.
    tgt[4] = ex.command().q[4] + 0.1;
    for (int i = 0; i < 2000; ++i) {
        now += kLoopPeriodSec;
        PublishServoTarget(*shm, tgt, now);
        ex.Cycle(now);
    }
    Check(!ex.servo_stale(), "no longer stale once fed again");
    Close(ex.command().q[4], tgt[4], 2e-4, "resumes tracking");
}

/** The two channels are mutually exclusive, in both directions. */
void TestChannelsAreMutuallyExclusive()
{
    std::printf("a trajectory and a servo stream cannot both drive the arm\n");
    {
        auto shm = FreshShm();
        FakeRobot robot;
        RtExecutor<FakeRobot> ex;
        double hold[kMaxDof] = {};
        ex.Init(shm.get(), &robot, kDof, WideLimits(), hold);
        shm->servo_enable.store(1u);
        ex.Cycle(0.0);
        Check(ex.state() == ExecState::kServoing, "servoing");

        // A goal published behind the bridge's back.
        FillRamp(shm->slots[0], 7, 0.0, 0.3, 1.0, 20);
        shm->published_slot.store(0);
        shm->publish_seq.store(1);
        ex.Cycle(kLoopPeriodSec);
        Check(ex.state() == ExecState::kServoing, "the stream keeps the arm");
        Check(ex.reject_reason() == RejectReason::kServoActive, "and says why the goal was dropped");
        Check(shm->adopted_seq.load() == 1, "the publish is still acknowledged, not left hanging");
    }
    {
        auto shm = FreshShm();
        FakeRobot robot;
        RtExecutor<FakeRobot> ex;
        double hold[kMaxDof] = {};
        ex.Init(shm.get(), &robot, kDof, WideLimits(), hold);

        FillRamp(shm->slots[0], 7, 0.0, 0.3, 1.0, 20);
        shm->published_slot.store(0);
        shm->publish_seq.store(1);
        ex.Cycle(0.0);
        Check(ex.state() == ExecState::kRunning, "running a trajectory");

        shm->servo_enable.store(1u);
        ex.Cycle(kLoopPeriodSec);
        Check(ex.state() == ExecState::kRunning, "servo mode does not interrupt it");
        Check(ex.reject_reason() == RejectReason::kTrajectoryActive, "and says why");
    }
}

/** The waist is pinned unless rt_server was told to command it, and a producer
 *  streaming an arm group has no business moving it. */
void TestServoCannotMoveThePinnedWaist()
{
    std::printf("servo targets cannot move the waist unless --control-waist\n");
    auto shm = FreshShm();
    shm->n_external = 2;
    shm->control_waist = 0u;
    FakeRobot robot;
    RtExecutor<FakeRobot> ex;
    double hold[kMaxDof] = {};
    hold[0] = 0.15;
    hold[1] = -0.25;
    ex.Init(shm.get(), &robot, kDof, WideLimits(), hold);
    shm->servo_enable.store(1u);

    double now = 0.0;
    double tgt[kMaxDof] = {};
    tgt[0] = 1.0;    // both waist axes asked to move a long way...
    tgt[1] = 1.0;
    tgt[5] = 0.2;    // ...alongside a legitimate arm joint
    for (int i = 0; i < 3000; ++i) {
        now += kLoopPeriodSec;
        PublishServoTarget(*shm, tgt, now);
        ex.Cycle(now);
    }
    Close(ex.command().q[0], 0.15, 1e-12, "waist axis 0 did not move at all");
    Close(ex.command().q[1], -0.25, 1e-12, "waist axis 1 did not move at all");
    Close(ex.command().q[5], 0.2, 2e-4, "the arm joint in the same target still moved");
}

/** A target that jumps further than max_jump_rad is a glitch, and refusing it
 *  has to be visible or it presents as the arm mysteriously not following. */
void TestServoRejectsAJumpAndCountsIt()
{
    std::printf("a servo target that jumps is refused, and the count is published\n");
    auto shm = FreshShm();
    FakeRobot robot;
    RtExecutor<FakeRobot> ex;
    double hold[kMaxDof] = {};
    ex.Init(shm.get(), &robot, kDof, WideLimits(), hold);
    shm->servo_enable.store(1u);

    double now = 0.0;
    double tgt[kMaxDof] = {};
    tgt[3] = 0.1;
    for (int i = 0; i < 2000; ++i) {
        now += kLoopPeriodSec;
        PublishServoTarget(*shm, tgt, now);
        ex.Cycle(now);
    }
    Check(ex.servo_rejected() == 0, "nothing refused so far");

    double wild[kMaxDof] = {};
    wild[3] = 4.0;    // an IK branch flip, or a clutch that forgot to re-seed
    now += kLoopPeriodSec;
    PublishServoTarget(*shm, wild, now);
    ex.Cycle(now);
    Check(ex.servo_rejected() == 1, "the refusal is counted");

    for (int i = 0; i < 2000; ++i) {
        now += kLoopPeriodSec;
        ex.Cycle(now);   // deliberately not refreshing: the stamp is now stale
    }
    // Within a milliradian of the last good target. The point is that it is
    // nowhere near the 4.0 rad that was asked for, not the exact convergence
    // -- test_servo_tracker covers that against realistic joint limits.
    Close(ex.command().q[3], 0.1, 1e-3, "the arm stayed where the last good target put it");
    Check(ex.command().q[3] < 0.2, "and nowhere near the target that was refused");

    RtStatus st{};
    Check(ReadStatus(*shm, st), "status is readable");
    Check(st.servo_rejected == 1, "and carries the count across the mapping");
}

void TestIdleHoldsCapturedPosition()
{
    std::printf("idle streams the captured hold position, continuously\n");
    auto shm = FreshShm();
    FakeRobot robot;
    RtExecutor<FakeRobot> ex;
    double hold[kMaxDof] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
    ex.Init(shm.get(), &robot, kDof, WideLimits(), hold);

    for (int i = 0; i < 10; ++i) {
        ex.Cycle(i * kLoopPeriodSec);
    }
    Check(robot.streamed.size() == 10, "a command every cycle, none skipped");
    for (const auto& s : robot.streamed) {
        for (std::uint32_t j = 0; j < kDof; ++j) {
            Close(s.q[j], hold[j], 1e-12, "holds the captured position");
            Close(s.dq[j], 0.0, 1e-12, "zero velocity while idle");
        }
    }
    Check(ex.state() == ExecState::kIdle, "state is idle");
}

void TestAdoptAndRunToCompletion()
{
    std::printf("adopts a published trajectory, runs it, then holds the end\n");
    auto shm = FreshShm();
    FakeRobot robot;
    RtExecutor<FakeRobot> ex;
    double hold[kMaxDof] = {};
    ex.Init(shm.get(), &robot, kDof, WideLimits(), hold);

    const double T = 0.5;
    const std::uint32_t slot = PickFreeSlot(*shm);
    FillRamp(shm->slots[slot], 77, 0.0, 1.0, T, 22);
    const std::uint64_t seq = PublishSlot(*shm, slot);

    double t = 0.0;
    ex.Cycle(t);  // adoption happens on the first cycle that sees the publish
    Check(ex.state() == ExecState::kRunning, "adopted and running");
    Check(shm->adopted_seq.load() == seq, "acknowledged the publish sequence");
    Check(shm->reading_slot.load() == slot, "marked the slot as in use");
    Check(shm->status.active_id == 77, "reported the goal id");

    // Run past the end.
    for (int i = 1; i <= 700; ++i) {
        t = i * kLoopPeriodSec;
        ex.Cycle(t);
    }
    Check(ex.state() == ExecState::kFinished, "finished at the end of the trajectory");
    Close(ex.command().q[0], 1.0, 1e-6, "ended at the final position");
    Close(ex.command().dq[0], 0.0, 1e-12, "ended at rest");

    // Monotonic progress, and never beyond the commanded range.
    double prev = -1e9;
    bool monotonic = true, bounded = true;
    for (const auto& s : robot.streamed) {
        monotonic = monotonic && s.q[0] >= prev - 1e-9;
        bounded = bounded && s.q[0] >= -1e-9 && s.q[0] <= 1.0 + 1e-9;
        prev = s.q[0];
    }
    Check(monotonic, "commanded position advances monotonically on a ramp");
    Check(bounded, "commanded position stays inside the planned range");
}

void TestNoPositionJumpAtAdoption()
{
    std::printf("adoption does not jump the commanded position\n");
    // The first streamed command after adoption must be essentially where the
    // last one was, or the robot takes a step input. This is why the bridge
    // rejects a goal whose first point is away from the measured position.
    auto shm = FreshShm();
    FakeRobot robot;
    RtExecutor<FakeRobot> ex;
    double hold[kMaxDof] = {0.25};
    ex.Init(shm.get(), &robot, kDof, WideLimits(), hold);
    ex.Cycle(0.0);
    const double before = ex.command().q[0];

    const std::uint32_t slot = PickFreeSlot(*shm);
    FillRamp(shm->slots[slot], 1, 0.25, 0.75, 0.4, 20);  // starts where we are
    PublishSlot(*shm, slot);
    ex.Cycle(kLoopPeriodSec);
    Close(ex.command().q[0], before, 1e-9, "no step at the moment of adoption");
}

void TestClampingHonoursLimits()
{
    std::printf("clamping bounds position, velocity and acceleration\n");
    auto shm = FreshShm();
    FakeRobot robot;
    RtExecutor<FakeRobot> ex;
    Limits tight = WideLimits();
    tight.q_max[0] = 0.3;       // the ramp wants to reach 1.0
    tight.dq_max[0] = 0.5;
    tight.ddq_max[0] = 2.0;
    double hold[kMaxDof] = {};
    ex.Init(shm.get(), &robot, kDof, tight, hold);

    const std::uint32_t slot = PickFreeSlot(*shm);
    FillRamp(shm->slots[slot], 2, 0.0, 1.0, 0.3, 20);
    PublishSlot(*shm, slot);
    for (int i = 0; i <= 400; ++i) {
        ex.Cycle(i * kLoopPeriodSec);
    }
    bool within = true;
    for (const auto& s : robot.streamed) {
        within = within && s.q[0] <= tight.q_max[0] + 1e-12
                        && s.q[0] >= tight.q_min[0] - 1e-12
                        && std::fabs(s.dq[0]) <= tight.dq_max[0] + 1e-12
                        && std::fabs(s.ddq[0]) <= tight.ddq_max[0] + 1e-12;
    }
    Check(within, "every streamed command respects the limits");
}

void TestCancelDeceleratesRatherThanStepping()
{
    std::printf("cancel decelerates to rest instead of stepping the velocity\n");
    auto shm = FreshShm();
    FakeRobot robot;
    RtExecutor<FakeRobot> ex;
    Limits lim = WideLimits();
    lim.ddq_max[0] = 5.0;
    double hold[kMaxDof] = {};
    ex.Init(shm.get(), &robot, kDof, lim, hold);

    const std::uint32_t slot = PickFreeSlot(*shm);
    FillRamp(shm->slots[slot], 3, 0.0, 2.0, 1.0, 40);
    PublishSlot(*shm, slot);
    for (int i = 0; i <= 500; ++i) {  // mid-motion, so velocity is non-zero
        ex.Cycle(i * kLoopPeriodSec);
    }
    const double v_at_cancel = ex.command().dq[0];
    Check(std::fabs(v_at_cancel) > 0.1, "was actually moving when cancelled");

    shm->cancel_request.store(1u);
    const std::size_t first = robot.streamed.size();
    for (int i = 501; i <= 2000; ++i) {
        ex.Cycle(i * kLoopPeriodSec);
    }
    Check(ex.state() == ExecState::kAborted, "ended up stopped");
    Close(ex.command().dq[0], 0.0, 1e-9, "came to rest");

    // No velocity step larger than ddq_max * dt anywhere after the cancel, and
    // position stays continuous.
    double worst_dv = 0.0, worst_dq = 0.0;
    for (std::size_t i = first; i + 1 < robot.streamed.size(); ++i) {
        worst_dv = std::fmax(worst_dv,
            std::fabs(robot.streamed[i + 1].dq[0] - robot.streamed[i].dq[0]));
        worst_dq = std::fmax(worst_dq,
            std::fabs(robot.streamed[i + 1].q[0] - robot.streamed[i].q[0]));
    }
    const double allowed_dv = lim.ddq_max[0] * kLoopPeriodSec * 1.001;
    std::printf("  worst velocity step after cancel %.6f (allowed %.6f)\n", worst_dv, allowed_dv);
    Check(worst_dv <= allowed_dv, "velocity never steps beyond ddq_max * dt");
    Check(worst_dq <= lim.dq_max[0] * kLoopPeriodSec * 1.001, "position stays continuous");
}

void TestFaultStopsAndReports()
{
    std::printf("a fault aborts, asks for shutdown and stops streaming\n");
    auto shm = FreshShm();
    FakeRobot robot;
    RtExecutor<FakeRobot> ex;
    double hold[kMaxDof] = {};
    ex.Init(shm.get(), &robot, kDof, WideLimits(), hold);
    ex.Cycle(0.0);
    Check(!ex.stop_requested(), "no shutdown requested while healthy");

    robot.faulted = true;
    const std::size_t before = robot.streamed.size();
    ex.Cycle(kLoopPeriodSec);
    Check(ex.state() == ExecState::kAborted, "aborted on fault");
    Check(ex.stop_requested(), "shutdown requested");
    Check(robot.streamed.size() == before, "nothing streamed into a faulted robot");
    RtStatus st{};
    Check(ReadStatus(*shm, st), "status still readable");
    Check(st.exec_state == static_cast<std::uint32_t>(ExecState::kAborted),
        "abort reported through shared memory");
}

void TestRejectsMalformedSlot()
{
    std::printf("a malformed slot is rejected, not executed\n");
    auto shm = FreshShm();
    FakeRobot robot;
    RtExecutor<FakeRobot> ex;
    double hold[kMaxDof] = {0.4};
    ex.Init(shm.get(), &robot, kDof, WideLimits(), hold);

    const std::uint32_t slot = PickFreeSlot(*shm);
    FillRamp(shm->slots[slot], 9, 0.4, 0.9, 0.3, 10);
    shm->slots[slot].n_joints = 3;  // wrong DoF
    const std::uint64_t seq = PublishSlot(*shm, slot);
    ex.Cycle(0.0);
    Check(ex.state() == ExecState::kRejected, "rejected the wrong-DoF slot");
    Check(shm->adopted_seq.load() == seq, "still acknowledged, so the bridge is not left waiting");
    Close(ex.command().q[0], 0.4, 1e-12, "kept holding after rejection");
    RtStatus st{};
    Check(ReadStatus(*shm, st), "status readable");
    Check(st.reject_reason == static_cast<std::uint32_t>(RejectReason::kDofMismatch),
        "reported the reason");
}

void TestSecondTrajectoryPreemptsFirst()
{
    std::printf("a second publish pre-empts the first\n");
    auto shm = FreshShm();
    FakeRobot robot;
    RtExecutor<FakeRobot> ex;
    double hold[kMaxDof] = {};
    ex.Init(shm.get(), &robot, kDof, WideLimits(), hold);

    const std::uint32_t a = PickFreeSlot(*shm);
    FillRamp(shm->slots[a], 100, 0.0, 1.0, 2.0, 30);
    PublishSlot(*shm, a);
    for (int i = 0; i <= 200; ++i) {
        ex.Cycle(i * kLoopPeriodSec);
    }
    Check(shm->status.active_id == 100, "running the first goal");

    const std::uint32_t b = PickFreeSlot(*shm);
    Check(b != a, "the second trajectory went to a different slot");
    FillRamp(shm->slots[b], 200, ex.command().q[0], 0.2, 0.5, 20);
    PublishSlot(*shm, b);
    ex.Cycle(201 * kLoopPeriodSec);
    Check(ex.state() == ExecState::kRunning, "still running");
    Check(shm->status.active_id == 200, "switched to the second goal");
    Check(shm->reading_slot.load() == b, "now reading the second slot");
}

void TestTimingStatsAreSane()
{
    std::printf("timing statistics count cycles and misses\n");
    auto shm = FreshShm();
    FakeRobot robot;
    RtExecutor<FakeRobot> ex;
    double hold[kMaxDof] = {};
    ex.Init(shm.get(), &robot, kDof, WideLimits(), hold);
    for (int i = 0; i < 1000; ++i) {
        ex.Cycle(i * kLoopPeriodSec);
    }
    Check(ex.cycles() == 1000, "counted every cycle");
    Check(ex.missed() == 0, "no misses on a perfect clock");
    Close(ex.mean_period(), kLoopPeriodSec, 1e-9, "mean period");

    ex.Cycle(1000 * kLoopPeriodSec + 0.004);  // a 5 ms gap
    Check(ex.missed() == 1, "a long gap counts as a miss");
}

}  // namespace

int main()
{
    std::printf("=== rt_executor ===\n");
    TestIdleHoldsCapturedPosition();
    TestAdoptAndRunToCompletion();
    TestNoPositionJumpAtAdoption();
    TestClampingHonoursLimits();
    TestCancelDeceleratesRatherThanStepping();
    TestFaultStopsAndReports();
    TestRejectsMalformedSlot();
    TestSecondTrajectoryPreemptsFirst();
    TestTimingStatsAreSane();
    TestServoModeTracksAStreamedTarget();
    TestStaleServoTargetBrakes();
    TestChannelsAreMutuallyExclusive();
    TestServoCannotMoveThePinnedWaist();
    TestServoRejectsAJumpAndCountsIt();
    std::printf("%s (%d failures)\n", g_failures == 0 ? "PASS" : "FAIL", g_failures);
    return g_failures == 0 ? 0 : 1;
}
