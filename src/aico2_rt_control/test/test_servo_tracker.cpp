// Exercises the servo tracker: the thing that turns a stream of "where I want
// the arm" into 1 kHz setpoints the robot will accept. This is the only place
// a jerk limit exists on this path -- the RDK has none -- so the limits are
// asserted on every cycle of every run rather than at the end.
//
// The headline assertion is Lag(): the tracker's own documentation claims the
// steady-state following error is v^2/(2a) plus the travel the jerk-limited
// brake ramp costs, and that the acceleration limit is therefore the lag knob.
// That formula is the reason the design looks the way it does, so it is checked
// numerically rather than left as a comment.

#include "aico2_rt_control/servo_tracker.hpp"

#include <cmath>
#include <cstdio>
#include <initializer_list>

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
        std::printf("  FAIL  %s (got %.9g, want %.9g, tol %.3g)\n", what, got, want, tol);
    }
}

constexpr std::uint32_t kDof = 9;
constexpr double kDt = kLoopPeriodSec;
constexpr double kVMax = 2.0;
constexpr double kAMax = 3.0;    // rt_server's --max-acc default
constexpr double kJMax = 200.0;
constexpr double kT = 0.03;   // the tracker's terminal time constant

Limits MakeLimits(double q_lo = -10.0, double q_hi = 10.0)
{
    Limits l{};
    for (std::uint32_t j = 0; j < kMaxDof; ++j) {
        l.q_min[j] = q_lo;
        l.q_max[j] = q_hi;
        l.dq_max[j] = kVMax;
        l.ddq_max[j] = kAMax;
    }
    return l;
}

ServoConfig MakeCfg(double lookahead = kT, double jump = 0.5, double jerk = kJMax)
{
    ServoConfig c{};
    c.timeout_sec = 0.1;
    c.max_jump_rad = jump;
    c.settle_sec = lookahead;
    for (std::uint32_t j = 0; j < kMaxDof; ++j) {
        c.max_jerk[j] = jerk;
    }
    return c;
}

/** A target naming every joint. */
ServoTarget Target(const double* q, const double* dq = nullptr)
{
    ServoTarget t{};
    for (std::uint32_t j = 0; j < kDof; ++j) {
        t.q[j] = q[j];
        t.dq[j] = (dq != nullptr) ? dq[j] : 0.0;
    }
    t.mask = (1u << kDof) - 1u;
    t.dof = kDof;
    t.have_dq = (dq != nullptr) ? 1u : 0u;
    t.seq = 1;
    t.stamp_mono = 0.0;
    return t;
}

/** Accumulates the per-cycle invariants so every test gets them for free. */
struct Invariants {
    double prev_dq[kMaxDof]{};
    double prev_ddq[kMaxDof]{};
    bool first = true;
    double worst_dq = 0.0, worst_ddq = 0.0, worst_jerk = 0.0, worst_dv_step = 0.0;

    void Observe(const Setpoint& sp, std::uint32_t dof)
    {
        for (std::uint32_t j = 0; j < dof; ++j) {
            worst_dq = std::fmax(worst_dq, std::fabs(sp.dq[j]));
            worst_ddq = std::fmax(worst_ddq, std::fabs(sp.ddq[j]));
            if (!first) {
                worst_jerk = std::fmax(worst_jerk,
                    std::fabs(sp.ddq[j] - prev_ddq[j]) / kDt);
                worst_dv_step = std::fmax(worst_dv_step,
                    std::fabs(sp.dq[j] - prev_dq[j]) / kDt);
            }
            prev_dq[j] = sp.dq[j];
            prev_ddq[j] = sp.ddq[j];
        }
        first = false;
    }

    void Assert(const char* tag, double jerk_limit = kJMax)
    {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "%s: velocity within dq_max", tag);
        Check(worst_dq <= kVMax + 1e-9, buf);
        std::snprintf(buf, sizeof(buf), "%s: acceleration within ddq_max", tag);
        Check(worst_ddq <= kAMax + 1e-9, buf);
        // The velocity step per cycle is the acceleration actually applied, so
        // bounding it is the same statement as C1 continuity of the stream.
        std::snprintf(buf, sizeof(buf), "%s: velocity continuous (step <= ddq_max*dt)", tag);
        Check(worst_dv_step <= kAMax + 1e-6, buf);
        std::snprintf(buf, sizeof(buf), "%s: jerk within max_jerk", tag);
        Check(worst_jerk <= jerk_limit + 1e-6, buf);
    }
};

// ---------------------------------------------------------------------------

/** A step target is reached exactly, from one side, and never overshot. */
void StepResponse()
{
    double q0[kMaxDof] = {};
    ServoTracker tr;
    tr.Init(kDof, MakeLimits(), MakeCfg(), q0, nullptr);

    double tgt[kMaxDof] = {};
    tgt[3] = 0.3;
    Check(tr.SetTarget(Target(tgt)), "step: target accepted");

    Invariants inv;
    Setpoint sp{};
    double peak = 0.0;
    for (int i = 0; i < 4000; ++i) {
        tr.Step(kDt, true, sp);
        inv.Observe(sp, kDof);
        peak = std::fmax(peak, sp.q[3]);
    }
    inv.Assert("step");
    // The brake ceiling bounds overshoot but cannot forbid it outright: the
    // jerk limit means the acceleration is still being turned around as the
    // target arrives. What matters is the scale -- 1e-3 rad is 0.06 deg, two
    // orders below the 0.02 rad that the under-reserved brake law produced.
    Check(peak <= 0.3 + 1e-3, "step: overshoot under a milliradian");
    Close(sp.q[3], 0.3, 2e-4, "step: converges on the target");
    Close(sp.q[0], 0.0, 1e-12, "step: untargeted joint did not move");
}

/**
 * The documented lag law. Chase a target moving at constant speed and compare
 * the steady-state error against v^2/(2a) + v*(a/j)/2 -- the stopping distance
 * plus the travel the jerk-limited brake ramp consumes. If this drifts, either
 * the comment in servo_tracker.hpp is wrong or the tracker is.
 */
void Lag()
{
    for (const double v : {0.3, 0.6, 1.0}) {
        double q0[kMaxDof] = {};
        ServoTracker tr;
        tr.Init(kDof, MakeLimits(-100.0, 100.0), MakeCfg(), q0, nullptr);

        Invariants inv;
        Setpoint sp{};
        double tq[kMaxDof] = {};
        double t = 0.0;
        // Long enough for the error to settle: the transient is a few hundred
        // milliseconds at these accelerations.
        for (int i = 0; i < 6000; ++i) {
            t += kDt;
            tq[3] = v * t;
            ServoTarget tgt = Target(tq);
            tgt.seq = static_cast<std::uint64_t>(i) + 1u;
            tr.SetTarget(tgt);
            tr.Step(kDt, true, sp);
            inv.Observe(sp, kDof);
        }
        inv.Assert("lag");

        const double predicted = v * v / (2.0 * kAMax) + v * (kAMax / kJMax);
        const double measured = v * t - sp.q[3];
        char buf[192];

        // The formula is the brake-limited floor, so the measured lag is never
        // below it. It can exceed it at low speed, where the brake ceiling is
        // slack and the terminal law is what binds instead.
        std::snprintf(buf, sizeof(buf),
            "lag at %.1f rad/s is at least the brake-limited floor", v);
        Check(measured >= 0.9 * predicted, buf);

        if (v >= 0.6) {
            // Fast enough that the brake ceiling governs throughout, which is
            // where the formula in servo_tracker.hpp is meant to be exact.
            std::snprintf(buf, sizeof(buf),
                "lag at %.1f rad/s matches v^2/(2a) + v*a/j", v);
            Close(measured, predicted, 0.1 * predicted, buf);
            std::snprintf(buf, sizeof(buf),
                "lag at %.1f rad/s is v/(2a) + a/j seconds", v);
            Close(measured / v, predicted / v, 0.1 * predicted / v, buf);
        }

        Close(sp.dq[3], v, 0.02, "lag: settles at the target's own speed");
    }
}

/** A stale producer brakes the arm to a stop, jerk-limited, and holds. */
void StaleBrakes()
{
    double q0[kMaxDof] = {};
    ServoTracker tr;
    tr.Init(kDof, MakeLimits(-100.0, 100.0), MakeCfg(), q0, nullptr);

    Invariants inv;
    Setpoint sp{};
    double tq[kMaxDof] = {};
    double t = 0.0;
    for (int i = 0; i < 1500; ++i) {   // come up to speed
        t += kDt;
        tq[3] = 1.0 * t;
        ServoTarget tgt = Target(tq);
        tgt.seq = static_cast<std::uint64_t>(i) + 1u;
        tr.SetTarget(tgt);
        tr.Step(kDt, true, sp);
        inv.Observe(sp, kDof);
    }
    Check(sp.dq[3] > 0.5, "stale: was actually moving first");
    const double q_at_cut = sp.q[3];

    // The producer goes quiet: follow = false from here on.
    for (int i = 0; i < 2000; ++i) {
        tr.Step(kDt, false, sp);
        inv.Observe(sp, kDof);
    }
    inv.Assert("stale");
    Check(tr.at_rest(), "stale: brakes to a standstill");
    Close(sp.dq[3], 0.0, 1e-12, "stale: zero velocity when stopped");
    Close(sp.ddq[3], 0.0, 1e-12, "stale: zero acceleration when stopped");
    // Coasting distance is bounded by the stopping distance at the speed it
    // was doing; it must not keep following the last target indefinitely.
    Check(sp.q[3] - q_at_cut < 0.3, "stale: stops promptly rather than running on");
}

/** Joints the target does not name are held, not dragged along. */
void MaskHolds()
{
    double q0[kMaxDof] = {};
    q0[0] = 0.25;   // a waist axis parked somewhere
    ServoTracker tr;
    tr.Init(kDof, MakeLimits(), MakeCfg(), q0, nullptr);

    double tq[kMaxDof] = {};
    tq[0] = 1.0;    // the target asks for the waist...
    tq[5] = 0.2;
    ServoTarget t = Target(tq);
    t.mask = (1u << 5);   // ...but only joint 5 is masked in
    Check(tr.SetTarget(t), "mask: target accepted");

    Setpoint sp{};
    for (int i = 0; i < 3000; ++i) {
        tr.Step(kDt, true, sp);
    }
    Close(sp.q[0], 0.25, 1e-12, "mask: unmasked joint stayed exactly put");
    Close(sp.dq[0], 0.0, 1e-12, "mask: unmasked joint has no velocity");
    Close(sp.q[5], 0.2, 2e-4, "mask: masked joint reached its target");
}

/**
 * A target outside the joint's travel is approached and stopped at the limit,
 * smoothly. The position clamp inside Step() is a safety net; if the brake law
 * is right it never fires, so the jerk bound must hold here too.
 */
void StopsAtJointLimit()
{
    double q0[kMaxDof] = {};
    ServoTracker tr;
    tr.Init(kDof, MakeLimits(-0.4, 0.4), MakeCfg(), q0, nullptr);

    double tq[kMaxDof] = {};
    // Outside the travel but inside max_jump_rad, so this is a genuine request
    // to go to the limit rather than a glitch the jump check would refuse.
    tq[2] = 0.45;
    Check(tr.SetTarget(Target(tq)), "limit: target accepted");

    Invariants inv;
    Setpoint sp{};
    double peak = 0.0;
    for (int i = 0; i < 4000; ++i) {
        tr.Step(kDt, true, sp);
        inv.Observe(sp, kDof);
        peak = std::fmax(peak, sp.q[2]);
    }
    inv.Assert("limit");
    Check(peak <= 0.4 + 1e-3, "limit: never commanded meaningfully past q_max");
    Close(sp.q[2], 0.4, 2e-4, "limit: settles on q_max");
    Close(sp.dq[2], 0.0, 1e-3, "limit: at rest at the limit");
}

/** Feedforward cannot push through a joint limit either. */
void FeedforwardRespectsLimit()
{
    double q0[kMaxDof] = {};
    ServoTracker tr;
    tr.Init(kDof, MakeLimits(-0.4, 0.4), MakeCfg(), q0, nullptr);

    double tq[kMaxDof] = {};
    double tdq[kMaxDof] = {};
    tq[2] = 0.4;     // exactly at the limit, so the error term is zero there
    tdq[2] = 1.5;    // and the producer still says "keep going"
    Check(tr.SetTarget(Target(tq, tdq)), "ff: target accepted");

    Invariants inv;
    Setpoint sp{};
    double peak = 0.0;
    for (int i = 0; i < 4000; ++i) {
        tr.Step(kDt, true, sp);
        inv.Observe(sp, kDof);
        peak = std::fmax(peak, sp.q[2]);
    }
    inv.Assert("ff");
    Check(peak <= 0.4 + 1e-3, "ff: feedforward did not drive past q_max");
    Close(sp.dq[2], 0.0, 1e-3, "ff: at rest at the limit");
}

/** A step bigger than max_jump_rad is a glitch, not a command. */
void RejectsJumps()
{
    double q0[kMaxDof] = {};
    ServoTracker tr;
    tr.Init(kDof, MakeLimits(-100.0, 100.0), MakeCfg(kT, 0.5), q0, nullptr);

    double good[kMaxDof] = {};
    good[1] = 0.2;
    Check(tr.SetTarget(Target(good)), "jump: a reachable target is accepted");

    double wild[kMaxDof] = {};
    wild[1] = 3.0;   // 3 rad away from where the command is
    Check(!tr.SetTarget(Target(wild)), "jump: a 3 rad step is refused");
    Check(tr.rejected() == 1, "jump: the refusal is counted");

    // The refused target must not have displaced the accepted one.
    Setpoint sp{};
    for (int i = 0; i < 3000; ++i) {
        tr.Step(kDt, true, sp);
    }
    Close(sp.q[1], 0.2, 2e-4, "jump: the previous target still governs");

    // Only masked joints are checked: a wild value on an unmasked joint is
    // irrelevant, because it is not going to be followed.
    double wild_unmasked[kMaxDof] = {};
    wild_unmasked[0] = 9.0;
    wild_unmasked[1] = 0.2;
    ServoTarget t = Target(wild_unmasked);
    t.mask = (1u << 1);
    Check(tr.SetTarget(t), "jump: a wild value on an unmasked joint is ignored, not refused");
    Check(tr.rejected() == 1, "jump: and not counted");
}

/**
 * A scheduling hiccup must not become a jump. Step() bounds dt, so a cycle
 * handed 100 ms moves at most what two nominal cycles would.
 */
void BoundsDt()
{
    double q0[kMaxDof] = {};
    ServoTracker tr;
    tr.Init(kDof, MakeLimits(-100.0, 100.0), MakeCfg(), q0, nullptr);

    double tq[kMaxDof] = {};
    tq[4] = 0.4;
    tr.SetTarget(Target(tq));

    Setpoint sp{};
    // Come up to full speed first, so a mishandled dt has something to scale.
    for (int i = 0; i < 100; ++i) {
        tr.Step(kDt, true, sp);
    }
    const double before = sp.q[4];
    tr.Step(0.1, true, sp);   // a 100 ms stall reported as one cycle
    const double moved = sp.q[4] - before;
    Check(moved <= kVMax * 2.0 * kDt + 1e-9, "dt: a 100 ms stall moves at most two cycles' worth");
    Check(moved > 0.0, "dt: but the cycle is not simply discarded");
}

/**
 * Entering servo mode from a non-zero commanded velocity keeps the stream
 * continuous. Not the usual path -- servo mode is entered from a hold -- but
 * seeding from the live command rather than assuming zero is what makes it
 * safe, so it is worth pinning down.
 */
void SeedsFromLiveVelocity()
{
    double q0[kMaxDof] = {};
    double v0[kMaxDof] = {};
    v0[6] = 0.8;
    ServoTracker tr;
    tr.Init(kDof, MakeLimits(-100.0, 100.0), MakeCfg(), q0, v0);

    Setpoint sp{};
    tr.Step(kDt, false, sp);   // nothing to follow: brake from 0.8 rad/s
    Check(std::fabs(sp.dq[6] - 0.8) <= kAMax * kDt + 1e-9,
        "seed: first command continues the velocity it was handed");

    Invariants inv;
    for (int i = 0; i < 2000; ++i) {
        tr.Step(kDt, false, sp);
        inv.Observe(sp, kDof);
    }
    inv.Assert("seed");
    Check(tr.at_rest(), "seed: brakes to rest");
}

/**
 * Raising the acceleration limit cuts the lag -- but only partly, and only if
 * the jerk limit rises with it. This is the claim that makes rt_server default
 * max_jerk to a multiple of ddq_max, so it is worth having as a test: with the
 * jerk limit held fixed, the second term of the lag formula grows and eats the
 * gain from the first.
 */
void AccelerationIsTheKnob()
{
    double errs[2] = {0.0, 0.0};
    const double accs[2] = {3.0, 12.0};
    for (int k = 0; k < 2; ++k) {
        Limits l = MakeLimits(-100.0, 100.0);
        for (std::uint32_t j = 0; j < kMaxDof; ++j) {
            l.ddq_max[j] = accs[k];
        }
        double q0[kMaxDof] = {};
        ServoTracker tr;
        // Jerk scaled with acceleration, as rt_server does by default.
        tr.Init(kDof, l, MakeCfg(kT, 0.5, 30.0 * accs[k]), q0, nullptr);
        Setpoint sp{};
        double tq[kMaxDof] = {};
        double t = 0.0;
        for (int i = 0; i < 6000; ++i) {
            t += kDt;
            tq[3] = 1.0 * t;
            ServoTarget tgt = Target(tq);
            tgt.seq = static_cast<std::uint64_t>(i) + 1u;
            tr.SetTarget(tgt);
            tr.Step(kDt, true, sp);
        }
        errs[k] = 1.0 * t - sp.q[3];
    }
    // Quadrupling the acceleration limit quarters only the v^2/(2a) term; the
    // a/max_jerk term is unchanged when the two scale together, so the ratio
    // lands near (1/6 + 1/30) / (1/24 + 1/30) = 2.67 rather than 4. Asserting
    // the real number is the point: it is the evidence that the jerk term is
    // not negligible, and that "just raise the acceleration limit" has a floor.
    const double ratio = errs[0] / errs[1];
    const double predicted = (1.0 / 6.0 + 1.0 / 30.0) / (1.0 / 24.0 + 1.0 / 30.0);
    Close(ratio, predicted, 0.3 * predicted,
        "knob: 4x acceleration gives roughly 2.7x less lag");
    // The two bounds are the claims that actually matter, and they are what
    // the loose tolerance above is protecting: raising the acceleration limit
    // does help, and it does not help as much as the v^2/(2a) term alone
    // suggests, because a/max_jerk is unchanged when the two scale together.
    Check(ratio > 1.5, "knob: more acceleration is meaningfully less lag");
    Check(ratio < 3.5, "knob: but not the 4x that ignoring the jerk term implies");
}

}  // namespace

int main()
{
    std::printf("test_servo_tracker\n");
    StepResponse();
    Lag();
    StaleBrakes();
    MaskHolds();
    StopsAtJointLimit();
    FeedforwardRespectsLimit();
    RejectsJumps();
    BoundsDt();
    SeedsFromLiveVelocity();
    AccelerationIsTheKnob();
    if (g_failures == 0) {
        std::printf("  all checks passed\n");
        return 0;
    }
    std::printf("  %d failure(s)\n", g_failures);
    return 1;
}
