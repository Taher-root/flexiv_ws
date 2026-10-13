// Verifies the sampler's maths. No robot, no ROS, no shared memory.
//
// The properties that matter are the boundary conditions and C2 continuity:
// if those hold, the resampled stream reproduces MoveIt's plan at the knots and
// never steps the acceleration, which is the entire reason for quintic.

#include "aico2_rt_control/traj_sampler.hpp"

#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

using namespace aico2_rt;

namespace {

int g_failures = 0;

void Check(bool ok, const char* what, double got = 0.0, double want = 0.0)
{
    if (!ok) {
        ++g_failures;
        std::printf("  FAIL  %s  (got %.12g, want %.12g)\n", what, got, want);
    }
}

void Close(double got, double want, double tol, const char* what)
{
    Check(std::fabs(got - want) <= tol, what, got, want);
}

/** A trajectory whose joint 0 follows a known analytic curve, so the sampler
 *  can be compared against truth rather than against itself. */
std::unique_ptr<Slot> MakeAnalytic(std::uint32_t n_points, double T, std::uint32_t dof)
{
    auto s = std::make_unique<Slot>();
    s->id = 42;
    s->n_points = n_points;
    s->n_joints = dof;
    for (std::uint32_t i = 0; i < n_points; ++i) {
        const double t = T * i / (n_points - 1);
        Point& p = s->points[i];
        p.t = t;
        for (std::uint32_t j = 0; j < dof; ++j) {
            // Smooth, non-polynomial, with a per-joint phase so joints differ.
            const double ph = 0.3 * j;
            p.q[j] = std::sin(t + ph);
            p.dq[j] = std::cos(t + ph);
            p.ddq[j] = -std::sin(t + ph);
        }
    }
    return s;
}

void TestKnotsAreExact()
{
    std::printf("knots reproduce the plan exactly\n");
    const std::uint32_t dof = 9;
    auto traj = MakeAnalytic(22, 4.0, dof);
    TrajSampler smp;
    smp.Reset(traj.get());

    Setpoint sp{};
    for (std::uint32_t i = 0; i < traj->n_points; ++i) {
        const Point& p = traj->points[i];
        // Nudge inside the last knot so Sample() interpolates rather than
        // taking the hold-at-end branch.
        const double t = (i + 1 == traj->n_points) ? p.t - 1e-12 : p.t;
        smp.Sample(t, sp);
        for (std::uint32_t j = 0; j < dof; ++j) {
            Close(sp.q[j], p.q[j], 1e-9, "q at knot");
            Close(sp.dq[j], p.dq[j], 1e-6, "dq at knot");
            Close(sp.ddq[j], p.ddq[j], 1e-4, "ddq at knot");
        }
    }
}

void TestC2Continuity()
{
    std::printf("acceleration is continuous across knots (C2)\n");
    const std::uint32_t dof = 9;
    auto traj = MakeAnalytic(22, 4.0, dof);
    const double eps = 1e-7;

    // Approach each interior knot from both sides and compare. A cubic would
    // fail this on ddq while passing on q and dq.
    for (std::uint32_t i = 1; i + 1 < traj->n_points; ++i) {
        const double tk = traj->points[i].t;
        TrajSampler a, b;
        a.Reset(traj.get());
        b.Reset(traj.get());
        Setpoint lo{}, hi{};
        a.Sample(tk - eps, lo);
        b.Sample(tk + eps, hi);
        for (std::uint32_t j = 0; j < dof; ++j) {
            Close(lo.q[j], hi.q[j], 1e-6, "q continuous");
            Close(lo.dq[j], hi.dq[j], 1e-4, "dq continuous");
            Close(lo.ddq[j], hi.ddq[j], 1e-2, "ddq continuous");
        }
    }
}

void TestDerivativesMatchFiniteDifference()
{
    std::printf("reported dq/ddq match the position curve's derivatives\n");
    const std::uint32_t dof = 3;
    auto traj = MakeAnalytic(15, 3.0, dof);
    const double h = 1e-5;

    for (double t = 0.2; t < 2.8; t += 0.137) {
        TrajSampler s0, s1, s2;
        s0.Reset(traj.get());
        s1.Reset(traj.get());
        s2.Reset(traj.get());
        Setpoint a{}, b{}, c{};
        s0.Sample(t - h, a);
        s1.Sample(t, b);
        s2.Sample(t + h, c);
        for (std::uint32_t j = 0; j < dof; ++j) {
            const double fd_v = (c.q[j] - a.q[j]) / (2 * h);
            const double fd_a = (c.q[j] - 2 * b.q[j] + a.q[j]) / (h * h);
            Close(b.dq[j], fd_v, 1e-5, "dq vs finite difference");
            Close(b.ddq[j], fd_a, 1e-2, "ddq vs finite difference");
        }
    }
}

void TestTracksAnalyticCurveBetweenKnots()
{
    std::printf("interpolation tracks the underlying curve between knots\n");
    // 22 points over 4 s is MoveIt's typical density. The quintic should follow
    // sin() to well under a milliradian there -- this is the number that says
    // resampling does not distort the plan.
    const std::uint32_t dof = 1;
    auto traj = MakeAnalytic(22, 4.0, dof);
    TrajSampler smp;
    smp.Reset(traj.get());
    double worst = 0.0;
    Setpoint sp;
    for (double t = 0.0; t < 4.0; t += 0.001) {
        smp.Sample(t, sp);
        worst = std::fmax(worst, std::fabs(sp.q[0] - std::sin(t)));
    }
    std::printf("  worst position error vs sin(t): %.3e rad\n", worst);
    Check(worst < 1e-6, "tracking error under 1e-6 rad", worst, 1e-6);
}

void TestHoldBeforeAndAfter()
{
    std::printf("holds the first point before the start and the last after\n");
    auto traj = MakeAnalytic(10, 2.0, 9);
    TrajSampler smp;
    smp.Reset(traj.get());
    Setpoint sp{};

    Check(smp.Sample(-1.0, sp), "sampling before the start stays in range");
    for (std::uint32_t j = 0; j < 9; ++j) {
        Close(sp.q[j], traj->points[0].q[j], 1e-12, "q held at first point");
        Close(sp.dq[j], 0.0, 1e-12, "dq zero before start");
        Close(sp.ddq[j], 0.0, 1e-12, "ddq zero before start");
    }

    const bool in_range = smp.Sample(99.0, sp);
    Check(!in_range, "sampling past the end reports finished");
    const Point& last = traj->points[traj->n_points - 1];
    for (std::uint32_t j = 0; j < 9; ++j) {
        Close(sp.q[j], last.q[j], 1e-12, "q held at last point");
        Close(sp.dq[j], 0.0, 1e-12, "dq zero after end");
    }
}

void TestForwardCursorCostIsBounded()
{
    std::printf("monotonic sampling never rewinds the cursor\n");
    // 4096 points sampled at 1 kHz: if the cursor searched, this would be
    // O(n^2). Just check it produces the right answer throughout, which it
    // cannot do if the cursor runs past a segment.
    auto traj = MakeAnalytic(kMaxPoints, 10.0, 2);
    TrajSampler smp;
    smp.Reset(traj.get());
    Setpoint sp{};
    double worst = 0.0;
    for (double t = 0.0; t < 10.0; t += 0.001) {
        smp.Sample(t, sp);
        worst = std::fmax(worst, std::fabs(sp.q[0] - std::sin(t)));
    }
    Check(worst < 1e-9, "dense trajectory still accurate", worst, 1e-9);
}

void TestEmptyAndDegenerate()
{
    std::printf("empty and degenerate inputs do not misbehave\n");
    TrajSampler smp;
    Setpoint sp{};
    smp.Reset(nullptr);
    Check(!smp.valid(), "null trajectory is invalid");
    Check(!smp.Sample(0.0, sp), "sampling a null trajectory returns false");

    auto empty = std::make_unique<Slot>();
    empty->n_points = 0;
    empty->n_joints = 9;
    smp.Reset(empty.get());
    Check(!smp.valid(), "zero-point trajectory is invalid");
    Check(!smp.Sample(0.0, sp), "sampling an empty trajectory returns false");

    // A zero-length segment must not divide by zero.
    auto degen = std::make_unique<Slot>();
    degen->n_points = 2;
    degen->n_joints = 1;
    degen->points[0].t = 0.0;
    degen->points[0].q[0] = 1.0;
    degen->points[1].t = 0.0;
    degen->points[1].q[0] = 2.0;
    Setpoint d{};
    TrajSampler::Interpolate(degen->points[0], degen->points[1], 0.0, 1, d);
    Check(std::isfinite(d.q[0]), "degenerate segment yields a finite value", d.q[0]);
}

void TestInvalidSampleLeavesOutputAlone()
{
    std::printf("an invalid trajectory leaves the caller's setpoint untouched\n");
    // Specified behaviour, not an accident: the RT task must fall back to the
    // position it captured, and a zeroed q would be a legal-looking command to
    // drive every joint to zero.
    Setpoint sp{};
    for (std::size_t j = 0; j < kMaxDof; ++j) {
        sp.q[j] = 1.25;
        sp.dq[j] = 2.5;
        sp.ddq[j] = 3.75;
    }
    TrajSampler smp;
    smp.Reset(nullptr);
    Check(!smp.Sample(0.0, sp), "invalid trajectory reports false");
    for (std::size_t j = 0; j < kMaxDof; ++j) {
        Close(sp.q[j], 1.25, 0.0, "q preserved");
        Close(sp.dq[j], 2.5, 0.0, "dq preserved");
        Close(sp.ddq[j], 3.75, 0.0, "ddq preserved");
    }
}

}  // namespace

int main()
{
    std::printf("=== traj_sampler ===\n");
    TestKnotsAreExact();
    TestC2Continuity();
    TestDerivativesMatchFiniteDifference();
    TestTracksAnalyticCurveBetweenKnots();
    TestHoldBeforeAndAfter();
    TestForwardCursorCostIsBounded();
    TestEmptyAndDegenerate();
    TestInvalidSampleLeavesOutputAlone();
    std::printf("%s (%d failures)\n", g_failures == 0 ? "PASS" : "FAIL", g_failures);
    return g_failures == 0 ? 0 : 1;
}
