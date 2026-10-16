"""Full inverse kinematics as a constrained optimisation, solved every frame.

This is the difference from ``moveit_servo``. Servo inverts the Jacobian
locally and, when a joint reaches its bound, halts -- it has no way to express
"use the other joints to get there". Here the joint limits are **constraints**,
so the solver returns a configuration inside them, redistributing motion across
the whole arm.

The formulation follows unitreerobotics/xr_teleoperate::

    minimize  w_pos * ||p(q) - p_target||^2
            + w_rot * ||log3(R(q) @ R_target.T)||^2
            + w_reg * ||q - q_nominal||^2
            + w_smooth * ||q - q_last||^2
    s.t.      lower <= q <= upper

with one deliberate change: xr_teleoperate regularises toward **zero**
(``sumsqr(q)``), which on an arm whose zero pose is singular biases the solver
into that singularity every frame. Regularising toward a nominal posture --
normally an SRDF named state -- is both safer and more general.

The problem is built ONCE and re-solved with new parameter values. Rebuilding
the NLP per frame would dominate the solve time.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import casadi as ca
import numpy as np

def rotation_cost(R, R_target):
    """Smooth, singularity-free orientation error for the NLP.

    This is the squared chordal (Frobenius) distance, ``||R - R_target||_F^2``,
    which equals ``4 (1 - cos theta)`` and so behaves like ``2 theta^2`` near
    convergence -- exactly the shape wanted -- while remaining perfectly smooth
    at every angle including 0 and pi.

    It replaces the textbook ``||log3(R R_target^T)||^2``, which is unusable
    here: ``log3`` needs ``acos``, whose derivative ``-1/sqrt(1 - x^2)`` is
    INFINITE when the rotation error reaches zero. Clamping the value does not
    help, because it is the derivative that blows up, and the optimiser lands on
    exactly that point every time it converges. In practice IPOPT reported
    "NaN detected for output grad_f_x" and silently stopped improving.
    """
    difference = R - R_target
    return ca.sumsqr(difference)


def rotation_angle(R, R_target) -> float:
    """Angle between two rotations, in radians. NUMERIC only -- for reporting.

    Kept out of the cost precisely because of the derivative problem above; it
    is only ever evaluated on a finished solution.
    """
    relative = np.asarray(R, float) @ np.asarray(R_target, float).T
    cos_theta = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return float(np.arccos(cos_theta))


@dataclass
class IKWeights:
    """Cost weights.

    Two deliberate departures from xr_teleoperate's ``50 / 1 / 0.02 / 0.1``:

    ``regularization`` there pulls toward **zero**; here it pulls toward a
    nominal posture, because an arm whose zero pose is singular would otherwise
    be biased into that singularity every frame.

    ``regularization`` is also lowered from 0.02 to 0.005. It is the SOLE
    source of steady-state tracking error; measured on the Piper, converged:

        w_reg    max position error    max orientation error
        0.000         0.011 mm              0.0008 deg
        0.005         0.584 mm              0.0480 deg
        0.020         2.272 mm              0.1869 deg
        0.100        10.092 mm              0.8398 deg

    Why so much lower than the reference: on a **6-DOF** arm a pose target fully
    determines the configuration (up to discrete branches), so there is no
    nullspace for a posture term to shape and it can only pull the solution away
    from the target. Its remaining value is real but narrow -- picking a
    consistent elbow branch, and behaving sensibly when the target is
    unreachable or near-singular -- and a small weight buys that without giving
    up 2 mm of accuracy. On a 7-DOF arm the same term would be nearly free and a
    larger weight would be appropriate.

    ``orientation`` is raised from 1 to 10. Measured on the Piper, converged:

        w_rot    position error    orientation error
          1         0.835 mm          0.782 deg
          5         0.901 mm          0.174 deg
         10         0.912 mm          0.088 deg
         50         0.922 mm          0.018 deg

    A 50:1 ratio leaves a steady-state orientation residual of ~0.8 deg, which
    is felt directly when twisting a gripper to align with something. Buying a
    9x improvement for 0.08 mm of position is worth it, and solve time is
    unchanged (0.65 ms at every weight tried). 10 keeps position clearly
    dominant while making the residual negligible.
    """

    position: float = 50.0
    orientation: float = 10.0
    regularization: float = 0.005
    smoothness: float = 0.1


@dataclass
class IKResult:
    q: np.ndarray
    success: bool
    solve_time: float
    position_error: float          # metres
    orientation_error: float       # radians
    status: str = ''
    limited_joints: list = field(default_factory=list)


class IKSolver:
    """Warm-started pose IK for one serial chain.

    Parameters
    ----------
    chain
        A :class:`moveit_teleop_ik.chain.SerialChain`.
    nominal
        Posture the solution is regularised toward. Pass an SRDF named state.
        **Do not pass the joint mid-range without checking it**: on the Piper it
        scores a Jacobian condition number of 34111, worse than the zero pose.
    limit_margin
        Kept inside the URDF limits so solutions do not land exactly on a bound,
        which is the state that leaves a differential controller unable to
        recover.
    planes
        Virtual walls, as ``(normal, offset)`` pairs enforcing
        ``normal . p(q) >= offset`` on the tool point, with ``normal`` in the
        planning frame. A table at z = 0.05 that the tool must stay above is
        ``([0, 0, 1], 0.05)``.

        These are constraints, not penalties, so the tool cannot cross them --
        and because the solver returns the closest feasible pose, pushing into
        a wall makes the tool SLIDE ALONG it rather than stop. That behaviour
        is a property of the formulation; no special-case code implements it.
    """

    def __init__(self, chain, nominal=None, weights: IKWeights = None,
                 limit_margin: float = 0.01, max_iter: int = 30,
                 tol: float = 1e-4, acceptable_tol: float = 5e-4,
                 acceptable_iter: int = 5, planes=None, verbose: bool = False):
        self.chain = chain
        self.weights = weights or IKWeights()
        self.max_iter = int(max_iter)

        nq = chain.nq
        self.nominal = (chain.midrange() if nominal is None
                        else np.asarray(nominal, float).reshape(nq))

        span = chain.upper - chain.lower
        margin = np.minimum(limit_margin, np.maximum(0.0, span / 2.0 - 1e-6))
        self.lower = chain.lower + margin
        self.upper = chain.upper - margin

        opti = ca.Opti()
        q = opti.variable(nq)

        # Targets and the previous solution are PARAMETERS, so the NLP is
        # constructed once here and only its numbers change per frame.
        p_target = opti.parameter(3)
        R_target = opti.parameter(3, 3)
        q_last = opti.parameter(nq)
        q_nominal = opti.parameter(nq)

        p, R = chain.fk(q)
        position_error = p - p_target

        w = self.weights
        # Smoothness is an Opti PARAMETER, not a baked constant, so one solve
        # can be retried with a much stronger pull toward the previous
        # solution. That retry is what turns a refused IK branch flip into a
        # nearby solution instead of a frozen arm: refusing alone leaves the
        # target unchanged, so the next solve simply rediscovers the branch
        # that was just rejected, and the arm sits still until the operator
        # moves away.
        self._w_smoothness = opti.parameter()
        opti.minimize(
            w.position * ca.sumsqr(position_error)
            + w.orientation * rotation_cost(R, R_target)
            + w.regularization * ca.sumsqr(q - q_nominal)
            + self._w_smoothness * ca.sumsqr(q - q_last))

        opti.subject_to(opti.bounded(self.lower, q, self.upper))

        # Virtual walls. One linear inequality each -- IPOPT handles them
        # natively, and sliding falls out of returning the closest feasible
        # pose.
        # Each wall's offset is an Opti PARAMETER, not a baked constant, so a
        # bounded wall can be switched off for a solve. A table is only there
        # while the tool is over it -- reach past its edge and an infinite
        # half-space would still hold the tool at table height across the whole
        # workspace. Rebuilding the NLP per solve would be the alternative and
        # is far too slow at 90 Hz; setting a parameter costs nothing.
        self.planes = []
        self._plane_offsets = []
        for normal, offset in (planes or []):
            normal = np.asarray(normal, float).reshape(3)
            length = np.linalg.norm(normal)
            if length < 1e-9:
                raise ValueError('plane normal must be non-zero')
            normal = normal / length
            self.planes.append((normal, float(offset)))
            parameter = opti.parameter()
            self._plane_offsets.append(parameter)
            # Written out rather than ca.dot(): inside Opti the FK expression
            # is an MX while a wrapped constant is an SX, and CasADi refuses to
            # mix the two. Plain arithmetic works for either.
            opti.subject_to(
                normal[0] * p[0] + normal[1] * p[1] + normal[2] * p[2]
                >= parameter)

        opti.solver('ipopt', {
            'print_time': False,
            'ipopt.print_level': 5 if verbose else 0,
            'ipopt.sb': 'yes',
            'ipopt.max_iter': self.max_iter,
            'ipopt.tol': tol,
            'ipopt.acceptable_tol': acceptable_tol,
            'ipopt.acceptable_iter': acceptable_iter,
            'ipopt.warm_start_init_point': 'yes',
        })

        self._opti = opti
        self._q = q
        self._p_target = p_target
        self._R_target = R_target
        self._q_last = q_last
        self._q_nominal = q_nominal
        self._position_error_fn = ca.Function(
            'perr', [q, p_target], [ca.norm_2(position_error)])
        self._rotation_fn = ca.Function('rot', [q], [R])

        self._last_solution = self.nominal.copy()

    # ---- solving ----------------------------------------------------------

    def reset(self, q=None) -> None:
        """Forget the warm start. Call when the arm has been moved by something
        else, so the next solve starts from where it actually is."""
        self._last_solution = (self.nominal.copy() if q is None
                               else self.chain.clamp(q))

    #: Offset used for a wall that is switched off. Far enough below anything
    #: the arm can reach that the constraint is inert, without being an
    #: infinity that IPOPT would have to special-case.
    INACTIVE_OFFSET = -1e3

    def solve(self, target_position, target_rotation, q_seed=None,
              active_planes=None, smoothness=None) -> IKResult:
        """Solve for a pose. ``target_rotation`` is a 3x3 matrix.

        ``active_planes`` is an optional per-plane boolean sequence. A wall
        that is off for this solve is not relaxed -- it is moved out of reach,
        which leaves the problem structure identical and the solve warm-started
        as usual.

        ``smoothness`` overrides the configured weight for this solve only.
        Raising it pulls the answer toward ``q_seed``, which is how a rejected
        branch flip is retried into a nearby solution.
        """
        seed = self._last_solution if q_seed is None else self.chain.clamp(q_seed)
        seed = np.clip(seed, self.lower, self.upper)

        opti = self._opti
        opti.set_value(
            self._w_smoothness,
            self.weights.smoothness if smoothness is None else float(smoothness))
        for index, parameter in enumerate(self._plane_offsets):
            on = True if active_planes is None else bool(active_planes[index])
            opti.set_value(parameter,
                           self.planes[index][1] if on else self.INACTIVE_OFFSET)
        opti.set_value(self._p_target, np.asarray(target_position, float).reshape(3))
        opti.set_value(self._R_target, np.asarray(target_rotation, float).reshape(3, 3))
        opti.set_value(self._q_last, seed)
        opti.set_value(self._q_nominal, self.nominal)
        opti.set_initial(self._q, seed)

        started = time.perf_counter()
        try:
            solution = opti.solve()
            q = np.array(solution.value(self._q)).reshape(-1)
            status = 'optimal'
            success = True
        except RuntimeError as exc:
            # IPOPT hitting max_iter still leaves a usable iterate, and for
            # teleoperation a slightly-off pose this frame beats no command at
            # all -- the next frame corrects it. Only a genuinely broken solve
            # should fall back.
            try:
                q = np.array(opti.debug.value(self._q)).reshape(-1)
                status = 'acceptable'
                success = np.all(np.isfinite(q))
            except Exception:  # noqa: BLE001
                q, status, success = seed.copy(), f'failed: {exc}', False
        elapsed = time.perf_counter() - started

        q = np.clip(q, self.lower, self.upper)
        if success:
            self._last_solution = q.copy()

        position_error = self._position_error_fn(
            q, np.asarray(target_position, float).reshape(3))
        orientation_error = rotation_angle(
            np.array(self._rotation_fn(q)),
            np.asarray(target_rotation, float).reshape(3, 3))

        return IKResult(
            q=q, success=success, solve_time=elapsed,
            position_error=float(position_error),
            orientation_error=orientation_error,
            status=status,
            limited_joints=self._at_limits(q))

    def plane_violations(self, q, tolerance: float = 1e-6) -> list:
        """Planes the given configuration violates. Should always be empty --
        they are constraints -- so anything here means an infeasible solve."""
        if not self.planes:
            return []
        position = self.chain.forward(q)[:3, 3]
        return [i for i, (normal, offset) in enumerate(self.planes)
                if float(np.dot(normal, position)) < offset - tolerance]

    def _at_limits(self, q, tolerance: float = 1e-4) -> list:
        return [self.chain.joint_names[i] for i in range(self.chain.nq)
                if q[i] <= self.lower[i] + tolerance
                or q[i] >= self.upper[i] - tolerance]
