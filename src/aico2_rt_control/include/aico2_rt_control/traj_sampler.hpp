/**
 * @file traj_sampler.hpp
 * @brief Resample a MoveIt trajectory at 1 kHz. Allocation-free, branch-light.
 *
 * MoveIt delivers a trajectory sparsely -- around 22 points for a typical move
 * -- and the RT loop needs 1000 setpoints a second, so the interpolation is the
 * controller. Getting it right is the whole point of going RT: in NRT the
 * robot's own generator did this, with no jerk limit and no knowledge of when
 * the next command was due (docs/open_issues.md issue 1a).
 *
 * Quintic Hermite per segment, from (q, dq, ddq) at both ends. Quintic rather
 * than cubic because it is C2: position, velocity AND acceleration are
 * continuous across every knot. A cubic would match positions and velocities
 * but step the acceleration at each waypoint, which is exactly the
 * discontinuity that made execution jerky under TOTG before Ruckig -- there is
 * no sense in fixing it in the planner and then reintroducing it here.
 *
 * The six basis functions on s = (t - t0) / h, each satisfying one boundary
 * condition and zeroing the other five:
 *
 *   H0 = 1 - 10s^3 + 15s^4 -  6s^5      (q0)
 *   H1 = s -  6s^3 +  8s^4 -  3s^5      (dq0,  scaled by h)
 *   H2 = s^2/2 - 3s^3/2 + 3s^4/2 - s^5/2 (ddq0, scaled by h^2)
 *   H3 =     10s^3 - 15s^4 +  6s^5      (q1)
 *   H4 =     -4s^3 +  7s^4 -  3s^5      (dq1,  scaled by h)
 *   H5 =    s^3/2  -  s^4  + s^5/2      (ddq1, scaled by h^2)
 */

#ifndef AICO2_RT_CONTROL_TRAJ_SAMPLER_HPP
#define AICO2_RT_CONTROL_TRAJ_SAMPLER_HPP

#include "aico2_rt_control/shm_protocol.hpp"

#include <cstddef>

namespace aico2_rt {

/** One sampled setpoint, in the shape StreamJointPosition wants. */
struct Setpoint {
    double q[kMaxDof];
    double dq[kMaxDof];
    double ddq[kMaxDof];
};

/**
 * @brief Stateful sampler over one trajectory.
 *
 * Holds a segment index that only ever moves forward, because `t` is monotonic
 * in a control loop. That makes each sample O(1) amortised with no search, and
 * it is the reason this is a class rather than a free function.
 */
class TrajSampler {
public:
    /** Point at a trajectory. Resets the cursor. `traj` must outlive this. */
    void Reset(const Slot* traj)
    {
        traj_ = traj;
        seg_ = 0;
        n_joints_ = (traj != nullptr) ? traj->n_joints : 0;
    }

    bool valid() const { return traj_ != nullptr && traj_->n_points > 0; }
    std::uint32_t n_joints() const { return n_joints_; }

    /** Total duration, i.e. the last point's time_from_start. */
    double duration() const
    {
        return valid() ? traj_->points[traj_->n_points - 1].t : 0.0;
    }

    /**
     * @brief Sample the trajectory at time `t`.
     * @return true while `t` is still inside the trajectory.
     *
     * @warning When this returns false because there is no valid trajectory,
     * `out` is left untouched. Do not stream it. The caller must fall back to
     * the position it captured when it entered RT mode. `out` is deliberately
     * not zeroed in that case: a zero joint vector is a legal-looking command
     * that would drive every axis to zero, which is far worse than a value the
     * caller knows to ignore.
     *
     * Returning false at the natural end of a trajectory is different -- there
     * `out` holds the final point with zero rates, and streaming it is exactly
     * right.
     */
    bool Sample(double t, Setpoint& out)
    {
        if (!valid()) {
            return false;  // out untouched, by contract
        }
        const std::uint32_t n = traj_->n_points;

        // Strictly before the start: hold the first point, which is where the
        // robot already is -- the bridge rejects a goal whose start does not
        // match. Note `<`, not `<=`: at exactly t0 the trajectory is evaluated,
        // so the plan's own dq and ddq are reproduced rather than replaced with
        // zeros. For a trajectory that satisfies Flexiv's zero-velocity-start
        // requirement those are zero anyway, but the sampler should not be the
        // thing deciding that.
        if (t < traj_->points[0].t) {
            Hold(traj_->points[0], out);
            return true;
        }
        // At or past the end: hold the last point with zero rates. Flexiv
        // require a continuous stream, so stopping is holding, not silence.
        // This is only continuous if the trajectory ends at rest; MoveIt's do,
        // and the bridge rejects one that does not.
        if (t >= traj_->points[n - 1].t) {
            Hold(traj_->points[n - 1], out);
            return false;
        }
        // Forward-only cursor. `t` is monotonic, so this advances at most a
        // few steps per call and never searches backwards.
        while (seg_ + 1 < n - 1 && t >= traj_->points[seg_ + 1].t) {
            ++seg_;
        }
        const Point& p0 = traj_->points[seg_];
        const Point& p1 = traj_->points[seg_ + 1];
        Interpolate(p0, p1, t, n_joints_, out);
        return true;
    }

    /** Quintic Hermite between two points. Exposed for testing. */
    static void Interpolate(
        const Point& p0, const Point& p1, double t, std::uint32_t n_joints, Setpoint& out)
    {
        const double h = p1.t - p0.t;
        if (h <= 0.0) {
            // Degenerate segment; the bridge rejects non-monotonic time, so
            // this is belt and braces rather than an expected path.
            Hold(p1, out);
            return;
        }
        const double s = (t - p0.t) / h;
        const double s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;

        // Position basis.
        const double H0 = 1.0 - 10.0 * s3 + 15.0 * s4 - 6.0 * s5;
        const double H1 = s - 6.0 * s3 + 8.0 * s4 - 3.0 * s5;
        const double H2 = 0.5 * s2 - 1.5 * s3 + 1.5 * s4 - 0.5 * s5;
        const double H3 = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
        const double H4 = -4.0 * s3 + 7.0 * s4 - 3.0 * s5;
        const double H5 = 0.5 * s3 - s4 + 0.5 * s5;

        // d/ds of the above. Velocity is (1/h) * d/ds.
        const double G0 = -30.0 * s2 + 60.0 * s3 - 30.0 * s4;
        const double G1 = 1.0 - 18.0 * s2 + 32.0 * s3 - 15.0 * s4;
        const double G2 = s - 4.5 * s2 + 6.0 * s3 - 2.5 * s4;
        const double G3 = 30.0 * s2 - 60.0 * s3 + 30.0 * s4;
        const double G4 = -12.0 * s2 + 28.0 * s3 - 15.0 * s4;
        const double G5 = 1.5 * s2 - 4.0 * s3 + 2.5 * s4;

        // d2/ds2. Acceleration is (1/h^2) * d2/ds2.
        const double K0 = -60.0 * s + 180.0 * s2 - 120.0 * s3;
        const double K1 = -36.0 * s + 96.0 * s2 - 60.0 * s3;
        const double K2 = 1.0 - 9.0 * s + 18.0 * s2 - 10.0 * s3;
        const double K3 = 60.0 * s - 180.0 * s2 + 120.0 * s3;
        const double K4 = -24.0 * s + 84.0 * s2 - 60.0 * s3;
        const double K5 = 3.0 * s - 12.0 * s2 + 10.0 * s3;

        const double inv_h = 1.0 / h;
        const double inv_h2 = inv_h * inv_h;

        for (std::uint32_t j = 0; j < n_joints; ++j) {
            const double q0 = p0.q[j], v0 = p0.dq[j], a0 = p0.ddq[j];
            const double q1 = p1.q[j], v1 = p1.dq[j], a1 = p1.ddq[j];

            out.q[j] = H0 * q0 + H1 * h * v0 + H2 * h * h * a0
                     + H3 * q1 + H4 * h * v1 + H5 * h * h * a1;
            out.dq[j] = (G0 * q0 + G1 * h * v0 + G2 * h * h * a0
                       + G3 * q1 + G4 * h * v1 + G5 * h * h * a1) * inv_h;
            out.ddq[j] = (K0 * q0 + K1 * h * v0 + K2 * h * h * a0
                        + K3 * q1 + K4 * h * v1 + K5 * h * h * a1) * inv_h2;
        }
    }

private:
    static void Hold(const Point& p, Setpoint& out)
    {
        for (std::size_t j = 0; j < kMaxDof; ++j) {
            out.q[j] = p.q[j];
            out.dq[j] = 0.0;
            out.ddq[j] = 0.0;
        }
    }

    const Slot* traj_ = nullptr;
    std::uint32_t seg_ = 0;
    std::uint32_t n_joints_ = 0;
};

}  // namespace aico2_rt

#endif  // AICO2_RT_CONTROL_TRAJ_SAMPLER_HPP
