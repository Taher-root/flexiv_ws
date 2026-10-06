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
    double q_[kMaxDof]{}, dq_[kMaxDof]{}, tau_[kMaxDof]{};
    std::vector<Setpoint> streamed;

    bool fault() const { return faulted; }
    bool operational() const { return op; }
    const double* q() const { return q_; }
    const double* dq() const { return dq_; }
    const double* tau() const { return tau_; }
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
    return shm;
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
    Check(shm->state.active_id == 77, "reported the goal id");

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
    State st{};
    Check(ReadState(*shm, st), "state still readable");
    Check(st.fault == 1u, "fault reported through shared memory");
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
    State st{};
    Check(ReadState(*shm, st), "state readable");
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
    Check(shm->state.active_id == 100, "running the first goal");

    const std::uint32_t b = PickFreeSlot(*shm);
    Check(b != a, "the second trajectory went to a different slot");
    FillRamp(shm->slots[b], 200, ex.command().q[0], 0.2, 0.5, 20);
    PublishSlot(*shm, b);
    ex.Cycle(201 * kLoopPeriodSec);
    Check(ex.state() == ExecState::kRunning, "still running");
    Check(shm->state.active_id == 200, "switched to the second goal");
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
    std::printf("%s (%d failures)\n", g_failures == 0 ? "PASS" : "FAIL", g_failures);
    return g_failures == 0 ? 0 : 1;
}
