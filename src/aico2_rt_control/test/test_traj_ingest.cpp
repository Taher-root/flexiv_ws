// Goal validation and joint remapping. This is the gate that keeps a bad
// trajectory away from the 1 kHz loop, so each rejection reason gets a test.

#include "aico2_rt_control/traj_ingest.hpp"

#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

using namespace aico2_rt;

namespace {

int g_failures = 0;

void Expect(const RejectDetail& d, RejectReason want, const char* what)
{
    if (d.reason != want) {
        ++g_failures;
        std::printf("  FAIL  %s: got '%s', want '%s'\n", what, RejectReasonName(d.reason),
            RejectReasonName(want));
    } else if (want != RejectReason::kNone) {
        // The detail has to be usable, not merely present.
        std::printf("    (joint %u, point %u, value %.6g vs bound %.6g)\n", d.joint, d.point,
            d.value, d.bound);
    }
}
/** BuildJointMap has nothing per-joint to report, so it still returns a bare
 *  reason; both overloads exist so the tests read the same either way. */
void Expect(RejectReason got, RejectReason want, const char* what)
{
    if (got != want) {
        ++g_failures;
        std::printf("  FAIL  %s: got '%s', want '%s'\n", what, RejectReasonName(got),
            RejectReasonName(want));
    }
}

void Check(bool ok, const char* what)
{
    if (!ok) {
        ++g_failures;
        std::printf("  FAIL  %s\n", what);
    }
}

constexpr std::uint32_t kDof = 9;

/** The real names, waist first, as joint_map_probe confirmed. */
std::vector<std::string> RdkNames()
{
    return {"AGV_Joint1", "AGV_Joint2", "Left_joint1", "Left_joint2", "Left_joint3",
        "Left_joint4", "Left_joint5", "Left_joint6", "Left_joint7"};
}

IngestConfig MakeConfig(bool allow_waist = false)
{
    IngestConfig c;
    c.dof = kDof;
    c.n_external = 2;
    c.allow_waist_motion = allow_waist;
    for (std::uint32_t j = 0; j < kMaxDof; ++j) {
        c.limits.q_min[j] = -3.0;
        c.limits.q_max[j] = 3.0;
        c.limits.dq_max[j] = 2.0;
        c.limits.ddq_max[j] = 5.0;
    }
    return c;
}

/** A valid trajectory: starts at q_meas at rest, ends at rest. */
std::vector<Point> GoodTraj(const double* q_meas, double delta, int n = 20, double T = 2.0)
{
    std::vector<Point> pts(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        const double u = static_cast<double>(i) / (n - 1);
        const double e = 10 * u * u * u - 15 * u * u * u * u + 6 * u * u * u * u * u;
        const double de = (30 * u * u - 60 * u * u * u + 30 * u * u * u * u) / T;
        const double dde = (60 * u - 180 * u * u + 120 * u * u * u) / (T * T);
        Point& p = pts[static_cast<std::size_t>(i)];
        p.t = T * u;
        for (std::uint32_t j = 0; j < kMaxDof; ++j) {
            const bool moves = (j >= 2 && j < kDof);  // arm only
            p.q[j] = q_meas[j] + (moves ? delta * e : 0.0);
            p.dq[j] = moves ? delta * de : 0.0;
            p.ddq[j] = moves ? delta * dde : 0.0;
        }
    }
    return pts;
}

void TestJointMapReordersAndCatchesBadNames()
{
    std::printf("joint map: reorders, and rejects unknown or missing joints\n");
    const auto rdk = RdkNames();
    std::vector<int> map;

    // MoveIt's order: arm first, waist last. The map must invert that.
    std::vector<std::string> goal = {"Left_joint1", "Left_joint2", "Left_joint3", "Left_joint4",
        "Left_joint5", "Left_joint6", "Left_joint7", "AGV_Joint1", "AGV_Joint2"};
    Expect(BuildJointMap(goal, rdk, map), RejectReason::kNone, "a full goal in another order");
    Check(map.size() == 9, "map has one entry per goal joint");
    Check(map[0] == 2 && map[6] == 8, "arm joints map to indices 2..8");
    Check(map[7] == 0 && map[8] == 1, "waist joints map to indices 0 and 1");

    goal.push_back("Nonexistent_joint");
    Expect(BuildJointMap(goal, rdk, map), RejectReason::kUnknownJoint, "an unknown joint name");

    std::vector<std::string> partial = {"Left_joint1", "Left_joint2"};
    Expect(BuildJointMap(partial, rdk, map), RejectReason::kMissingJoint,
        "a goal covering only some joints");
}

void TestAcceptsAValidTrajectory()
{
    std::printf("a valid trajectory is accepted\n");
    double q[kMaxDof] = {0.0, 0.1, 0.2, -0.3, 0.4, 0.5, -0.1, 0.2, 0.3};
    const auto pts = GoodTraj(q, 0.4);
    Expect(ValidateTrajectory(pts.data(), static_cast<std::uint32_t>(pts.size()), q, MakeConfig()),
        RejectReason::kNone, "arm-only motion from rest to rest");
}

void TestRejectsStartPositionMismatch()
{
    std::printf("rejects a trajectory that does not start where the robot is\n");
    double q[kMaxDof] = {};
    auto pts = GoodTraj(q, 0.3);
    pts[0].q[4] += 0.5;  // well outside start_tolerance
    Expect(ValidateTrajectory(pts.data(), static_cast<std::uint32_t>(pts.size()), q, MakeConfig()),
        RejectReason::kStartPositionMismatch, "first point away from the measured position");

    // Within tolerance is fine: MoveIt plans from a /joint_states sample that
    // is always slightly stale, so exact equality can never be required.
    auto near = GoodTraj(q, 0.3);
    near[0].q[4] += 0.01;
    Expect(ValidateTrajectory(near.data(), static_cast<std::uint32_t>(near.size()), q,
               MakeConfig()),
        RejectReason::kNone, "a small start offset is tolerated");
}

void TestRejectsMotionNotAtRest()
{
    std::printf("rejects trajectories that do not start and end at rest\n");
    double q[kMaxDof] = {};
    auto a = GoodTraj(q, 0.3);
    a[0].dq[3] = 0.5;
    Expect(ValidateTrajectory(a.data(), static_cast<std::uint32_t>(a.size()), q, MakeConfig()),
        RejectReason::kNonZeroStartVelocity, "non-zero start velocity");

    auto b = GoodTraj(q, 0.3);
    b.back().dq[3] = 0.5;
    Expect(ValidateTrajectory(b.data(), static_cast<std::uint32_t>(b.size()), q, MakeConfig()),
        RejectReason::kNonZeroEndVelocity, "non-zero end velocity");
}

void TestRejectsBadTiming()
{
    std::printf("rejects non-monotonic or negative time\n");
    double q[kMaxDof] = {};
    auto a = GoodTraj(q, 0.2);
    a[5].t = a[4].t;  // duplicate timestamp
    Expect(ValidateTrajectory(a.data(), static_cast<std::uint32_t>(a.size()), q, MakeConfig()),
        RejectReason::kNonMonotonicTime, "a repeated timestamp");

    auto b = GoodTraj(q, 0.2);
    b[7].t = b[3].t - 0.1;  // goes backwards
    Expect(ValidateTrajectory(b.data(), static_cast<std::uint32_t>(b.size()), q, MakeConfig()),
        RejectReason::kNonMonotonicTime, "time running backwards");

    auto c = GoodTraj(q, 0.2);
    c[0].t = -0.5;
    Expect(ValidateTrajectory(c.data(), static_cast<std::uint32_t>(c.size()), q, MakeConfig()),
        RejectReason::kNonMonotonicTime, "negative first timestamp");
}

void TestRejectsLimitViolations()
{
    std::printf("rejects limit violations in position, velocity and acceleration\n");
    double q[kMaxDof] = {};
    auto cfg = MakeConfig();

    auto a = GoodTraj(q, 0.2);
    a[9].q[5] = 99.0;
    Expect(ValidateTrajectory(a.data(), static_cast<std::uint32_t>(a.size()), q, cfg),
        RejectReason::kLimitExceeded, "position beyond q_max");

    auto b = GoodTraj(q, 0.2);
    b[9].dq[5] = 50.0;
    Expect(ValidateTrajectory(b.data(), static_cast<std::uint32_t>(b.size()), q, cfg),
        RejectReason::kLimitExceeded, "velocity beyond dq_max");

    auto c = GoodTraj(q, 0.2);
    c[9].ddq[5] = 500.0;
    Expect(ValidateTrajectory(c.data(), static_cast<std::uint32_t>(c.size()), q, cfg),
        RejectReason::kLimitExceeded, "acceleration beyond ddq_max");
}

void TestWaistMotionGatedOnTheServerFlag()
{
    std::printf("waist motion is refused unless the server is commanding the waist\n");
    double q[kMaxDof] = {};
    // Same trajectory, but moving a waist axis as well.
    auto pts = GoodTraj(q, 0.3);
    for (auto& p : pts) {
        p.q[1] = q[1] + 0.4 * (p.t / pts.back().t);
    }
    Expect(ValidateTrajectory(pts.data(), static_cast<std::uint32_t>(pts.size()), q,
               MakeConfig(false)),
        RejectReason::kWaistMotionNotAllowed, "refused with the waist off");
    // With the waist allowed it is only the rest-at-both-ends rule that this
    // crude ramp breaks, so check it gets past the waist gate specifically.
    const RejectDetail with_waist = ValidateTrajectory(
        pts.data(), static_cast<std::uint32_t>(pts.size()), q, MakeConfig(true));
    Check(with_waist.reason != RejectReason::kWaistMotionNotAllowed,
        "the waist gate no longer fires with --control-waist");
}

void TestEmptyAndOversized()
{
    std::printf("rejects empty and oversized trajectories\n");
    double q[kMaxDof] = {};
    const auto pts = GoodTraj(q, 0.1);
    Expect(ValidateTrajectory(pts.data(), 0, q, MakeConfig()), RejectReason::kTooManyPoints,
        "zero points");
    Expect(ValidateTrajectory(pts.data(), kMaxPoints + 1, q, MakeConfig()),
        RejectReason::kTooManyPoints, "more points than a slot holds");
}

void TestFillSlotRoundTrips()
{
    std::printf("a validated trajectory round-trips into a slot\n");
    double q[kMaxDof] = {0.1, 0.2, 0.3};
    const auto pts = GoodTraj(q, 0.25, 30, 1.5);
    auto slot = std::make_unique<Slot>();
    FillSlot(*slot, 1234, pts.data(), static_cast<std::uint32_t>(pts.size()), kDof);

    Check(slot->id == 1234, "goal id copied");
    Check(slot->n_points == pts.size(), "point count copied");
    Check(slot->n_joints == kDof, "joint count copied");
    bool same = true;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        same = same && slot->points[i].t == pts[i].t;
        for (std::uint32_t j = 0; j < kDof; ++j) {
            same = same && slot->points[i].q[j] == pts[i].q[j]
                && slot->points[i].dq[j] == pts[i].dq[j]
                && slot->points[i].ddq[j] == pts[i].ddq[j];
        }
    }
    Check(same, "every point copied exactly");
}

/** The gate has to actually protect the executor, not just report. */
void TestValidatedTrajectoryExecutesCleanly()
{
    std::printf("an accepted trajectory runs through the executor without clamping\n");
    struct Fake {
        bool fault() const { return false; }
        bool operational() const { return true; }
        void Stream(const double*, const double*, const double*) {}
    } fake;

    auto shm = std::make_unique<Shm>();
    shm->published_slot.store(kNoSlot);
    shm->reading_slot.store(kNoSlot);
    double q[kMaxDof] = {0.0, 0.1, 0.2, -0.3, 0.4, 0.5, -0.1, 0.2, 0.3};
    const auto cfg = MakeConfig();
    const auto pts = GoodTraj(q, 0.4);
    Expect(ValidateTrajectory(pts.data(), static_cast<std::uint32_t>(pts.size()), q, cfg),
        RejectReason::kNone, "precondition: the trajectory is valid");

    const std::uint32_t slot = PickFreeSlot(*shm);
    FillSlot(shm->slots[slot], 5, pts.data(), static_cast<std::uint32_t>(pts.size()), kDof);
    PublishSlot(*shm, slot);

    RtExecutor<Fake> ex;
    ex.Init(shm.get(), &fake, kDof, cfg.limits, q);
    for (int i = 0; i <= 2500; ++i) {
        ex.Cycle(i * kLoopPeriodSec);
    }
    Check(ex.state() == ExecState::kFinished, "ran to completion");
    RtStatus st{};
    Check(ReadStatus(*shm, st), "status readable");
    Check(st.clamped == 0u, "no clamping was needed on a validated trajectory");
}

}  // namespace

int main()
{
    std::printf("=== traj_ingest ===\n");
    TestJointMapReordersAndCatchesBadNames();
    TestAcceptsAValidTrajectory();
    TestRejectsStartPositionMismatch();
    TestRejectsMotionNotAtRest();
    TestRejectsBadTiming();
    TestRejectsLimitViolations();
    TestWaistMotionGatedOnTheServerFlag();
    TestEmptyAndOversized();
    TestFillSlotRoundTrips();
    TestValidatedTrajectoryExecutesCleanly();
    std::printf("%s (%d failures)\n", g_failures == 0 ? "PASS" : "FAIL", g_failures);
    return g_failures == 0 ? 0 : 1;
}
