"""Tests for the IK NLP.

The test that justifies this whole package is
``test_reconfigures_instead_of_saturating_a_limit``: servo cannot produce that
behaviour, and it is the reason the arm jams today.
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from moveit_teleop_ik.chain import SerialChain
from moveit_teleop_ik.ik_solver import (
    IKSolver, IKWeights, rotation_angle, rotation_cost,
)

import casadi as ca


def piper_urdf():
    try:
        # Resolved through the package index, not an absolute path: a
        # hardcoded one exists only on the machine it was written on, so
        # everywhere else these tests skipped rather than failed -- silently
        # taking the real-robot coverage with them.
        share = get_package_share_directory('agx_arm_moveit')
        out = subprocess.run(
            ['xacro',
             os.path.join(share, 'config', 'agx_arm.urdf.xacro'),
             'arm_type:=piper', 'effector_type:=agx_gripper',
             'tcp_offset_xyz:=0 0 0.1425', 'tcp_offset_rpy:=0 0 0'],
            capture_output=True, text=True, timeout=60)
        return out.stdout if out.returncode == 0 and out.stdout else None
    except Exception:
        return None


URDF = piper_urdf()
pytestmark = pytest.mark.skipif(URDF is None, reason='Piper URDF not built')
READY = np.array([0.0, 1.65, -0.8, 0.0, 0.8, 0.0])


@pytest.fixture(scope='module')
def chain():
    return SerialChain(URDF, 'base_link', 'tcp_link')


@pytest.fixture(scope='module')
def solver(chain):
    return IKSolver(chain, nominal=READY)


# ---- log3 ------------------------------------------------------------------

def test_rotation_cost_is_zero_only_at_the_target():
    R = ca.SX.sym('R', 3, 3)
    Rt = ca.SX.sym('Rt', 3, 3)
    f = ca.Function('f', [R, Rt], [rotation_cost(R, Rt)])
    target = Rotation.from_euler('xyz', [0.3, -0.2, 0.9]).as_matrix()
    assert float(f(target, target)) == pytest.approx(0.0, abs=1e-12)
    for angle in (0.01, 0.5, 1.5, 3.0):
        R_off = Rotation.from_rotvec([0, 0, angle]).as_matrix() @ target
        assert float(f(R_off, target)) > 1e-6


def test_rotation_cost_gradient_is_finite_AT_the_solution():
    """The bug this replaced: log3 uses acos, whose derivative is INFINITE when
    the rotation error is exactly zero -- which is where the optimiser lands
    every time it converges. IPOPT reported NaN gradients and stalled."""
    R = ca.SX.sym('R', 3, 3)
    target = Rotation.from_euler('xyz', [0.2, 0.4, -0.1]).as_matrix()
    grad = ca.Function('g', [R], [ca.jacobian(rotation_cost(R, ca.SX(target)), R)])
    for R_at in (target,                                    # exactly converged
                 Rotation.from_rotvec([0, 0, 1e-12]).as_matrix() @ target,
                 Rotation.from_rotvec([0, 0, np.pi]).as_matrix() @ target):
        g = np.array(grad(R_at))
        assert np.all(np.isfinite(g)), 'non-finite gradient'


def test_rotation_angle_matches_scipy():
    """The reported error is a true angle in radians, even though the COST is
    the chordal distance."""
    rng = np.random.default_rng(0)
    for _ in range(200):
        target = Rotation.random(random_state=int(rng.integers(1 << 30))).as_matrix()
        rotvec = rng.normal(size=3)
        rotvec = rotvec / np.linalg.norm(rotvec) * rng.uniform(0, 3.0)
        R = Rotation.from_rotvec(rotvec).as_matrix() @ target
        assert rotation_angle(R, target) == pytest.approx(
            np.linalg.norm(rotvec), abs=1e-9)


# ---- accuracy --------------------------------------------------------------

def test_recovers_a_pose_generated_by_forward_kinematics(chain, solver):
    """The ground truth is exact: an FK pose is reachable by construction.

    Solved over successive frames rather than in one shot, because that is how
    the node runs it. The smoothness term deliberately limits how far a single
    solve may travel -- a one-frame leap to an arbitrary configuration is
    exactly the jerk this formulation exists to prevent.
    """
    rng = np.random.default_rng(1)
    for _ in range(10):
        q_true = np.clip(READY + rng.normal(scale=0.25, size=6),
                         chain.lower + 0.05, chain.upper - 0.05)
        T = chain.forward(q_true)
        solver.reset(READY)
        for _ in range(15):
            r = solver.solve(T[:3, 3], T[:3, :3])
        assert r.success
        # Both residuals are converged equilibria set by the cost weights, not
        # slow convergence -- they are identical at frame 15 and frame 80. The
        # regularization weight is what sets the position floor.
        assert r.position_error < 1e-3, f'{r.position_error*1000:.3f} mm'
        assert r.orientation_error < 2e-3, f'{np.degrees(r.orientation_error):.3f} deg'


def test_every_solution_respects_the_joint_limits(chain, solver):
    """Limits are constraints, not suggestions -- this is the whole point."""
    rng = np.random.default_rng(2)
    for _ in range(40):
        target = np.array([rng.uniform(0.0, 0.6), rng.uniform(-0.4, 0.4),
                           rng.uniform(-0.1, 0.6)])
        solver.reset(READY)
        r = solver.solve(target, np.eye(3))
        assert np.all(r.q >= chain.lower - 1e-9)
        assert np.all(r.q <= chain.upper + 1e-9)


def test_unreachable_target_returns_the_closest_feasible_pose(chain, solver):
    """Far outside the workspace: still in limits, still finite, no exception.

    Servo's answer to this is to stop entirely.
    """
    solver.reset(READY)
    r = solver.solve(np.array([5.0, 0.0, 0.0]), np.eye(3))
    assert np.all(np.isfinite(r.q))
    assert np.all(r.q >= chain.lower - 1e-9) and np.all(r.q <= chain.upper + 1e-9)
    assert r.position_error > 1.0          # honestly reports it did not get there


# ---- the behaviour servo cannot produce -----------------------------------

def test_reconfigures_instead_of_saturating_a_limit(chain):
    """Drive a target that needs the wrist to give up range to the arm.

    Seeded from a configuration with joint6 pushed hard against its bound, the
    solver must find a solution that pulls it back off the limit by using the
    other joints. Servo, which halts a joint at its bound and cannot recover
    one pushed past it, produces exactly the jam this replaces.
    """
    solver = IKSolver(chain, nominal=READY)
    reachable = chain.forward(READY)

    stuck = READY.copy()
    stuck[5] = chain.upper[5] - 1e-4        # joint6 pinned at its limit
    solver.reset(stuck)
    start_headroom = min(stuck[5] - chain.lower[5], chain.upper[5] - stuck[5])

    # 15 frames is ~170 ms at 90 Hz. Servo, by contrast, never recovers a
    # joint pushed to its bound: it halts it and the arm stays jammed.
    for _ in range(15):
        r = solver.solve(reachable[:3, 3], reachable[:3, :3])

    assert r.success
    assert r.position_error < 1e-3, f'{r.position_error*1000:.3f} mm'
    headroom = min(r.q[5] - chain.lower[5], chain.upper[5] - r.q[5])
    assert headroom > start_headroom + 0.5, (
        f'joint6 barely moved off its limit ({start_headroom:.3f} -> '
        f'{headroom:.3f} rad); the solver did not reconfigure')
    assert not r.limited_joints, f'ended with joints at limits: {r.limited_joints}'


def test_regularization_pulls_toward_the_nominal_posture(chain):
    """With position weight relaxed, the solution should drift to nominal."""
    weak = IKWeights(position=1.0, orientation=0.1, regularization=5.0,
                     smoothness=0.0)
    solver = IKSolver(chain, nominal=READY, weights=weak)
    solver.reset(chain.midrange())
    T = chain.forward(READY)
    r = solver.solve(T[:3, 3], T[:3, :3])
    assert np.linalg.norm(r.q - READY) < np.linalg.norm(chain.midrange() - READY)


def test_smoothness_keeps_consecutive_solutions_close(chain):
    """Consecutive nearby targets must not jump between IK branches."""
    solver = IKSolver(chain, nominal=READY)
    solver.reset(READY)
    previous = None
    base = chain.forward(READY)
    for i in range(40):
        target = base[:3, 3] + np.array([0.0, 0.002 * i, 0.0])
        r = solver.solve(target, base[:3, :3])
        if previous is not None:
            assert np.max(np.abs(r.q - previous)) < 0.15, f'branch jump at {i}'
        previous = r.q.copy()


# ---- diagnostics -----------------------------------------------------------

def test_orientation_weight_trades_against_position_as_documented(chain):
    """Pin the trade-off the default weights encode, so a later tweak is a
    deliberate act rather than an accident."""
    from moveit_teleop_ik.ik_solver import IKWeights
    T = chain.forward(READY + np.array([0.1, -0.1, 0.15, 0.2, -0.1, 0.3]))
    errors = {}
    for w_rot in (1.0, 10.0):
        s = IKSolver(chain, nominal=READY,
                     weights=IKWeights(position=50.0, orientation=w_rot))
        s.reset(READY)
        for _ in range(20):
            r = s.solve(T[:3, 3], T[:3, :3])
        errors[w_rot] = (r.position_error, r.orientation_error)

    # Raising the orientation weight must improve orientation a lot...
    assert errors[10.0][1] < errors[1.0][1] / 4
    # ...without meaningfully costing position.
    assert errors[10.0][0] < errors[1.0][0] + 5e-4


def test_regularization_is_the_source_of_steady_state_error(chain):
    """Pin the accuracy cost of the posture term.

    On a 6-DOF arm the pose fully determines the configuration, so this term can
    only pull away from the target. Copying xr_teleoperate's 0.02 would cost
    ~2 mm; the default trades most of that away while keeping branch selection.
    """
    from moveit_teleop_ik.ik_solver import IKWeights
    T = chain.forward(READY + np.array([0.2, -0.15, 0.2, 0.1, -0.2, 0.1]))
    errors = {}
    for w_reg in (0.0, 0.02):
        s = IKSolver(chain, nominal=READY, weights=IKWeights(regularization=w_reg))
        s.reset(READY)
        for _ in range(20):
            r = s.solve(T[:3, 3], T[:3, :3])
        errors[w_reg] = r.position_error

    assert errors[0.0] < 1e-4, 'with no posture term the solve should be exact'
    assert errors[0.02] > errors[0.0] * 10, 'posture term should visibly bias it'


def test_reports_solve_time_and_limited_joints(chain, solver):
    solver.reset(READY)
    T = chain.forward(READY)
    r = solver.solve(T[:3, 3], T[:3, :3])
    assert r.solve_time > 0
    assert isinstance(r.limited_joints, list)
    assert r.status in ('optimal', 'acceptable')


# ---- virtual walls ---------------------------------------------------------

def test_tool_cannot_cross_a_plane(chain):
    """A wall is a constraint, so the solution is on the legal side. Always."""
    floor_z = chain.forward(READY)[2, 3] - 0.02
    solver = IKSolver(chain, nominal=READY, planes=[([0, 0, 1], floor_z)])
    solver.reset(READY)

    below = chain.forward(READY)[:3, 3].copy()
    below[2] -= 0.25                       # target well underneath the floor
    for _ in range(25):
        r = solver.solve(below, chain.forward(READY)[:3, :3])

    assert r.q is not None
    reached = chain.forward(r.q)[:3, 3]
    assert reached[2] >= floor_z - 1e-4, (
        f'tool went {floor_z - reached[2]:.4f} m through the wall')
    assert not solver.plane_violations(r.q)


def test_tool_slides_along_the_wall_rather_than_stopping(chain):
    """The behaviour the constraint buys for free.

    Push down AND sideways: the tool should refuse the downward component but
    still make the lateral progress, because the closest feasible pose lies ON
    the plane rather than at the last legal point.
    """
    start = chain.forward(READY)[:3, 3]
    rotation = chain.forward(READY)[:3, :3]
    floor_z = start[2] - 0.02
    solver = IKSolver(chain, nominal=READY, planes=[([0, 0, 1], floor_z)])
    solver.reset(READY)

    lateral = 0.0
    for i in range(1, 40):
        target = start + np.array([0.0, 0.004 * i, -0.20])   # down and across
        r = solver.solve(target, rotation)
        reached = chain.forward(r.q)[:3, 3]
        assert reached[2] >= floor_z - 1e-4, 'crossed the wall while sliding'
        lateral = reached[1] - start[1]

    assert lateral > 0.05, (
        f'tool only moved {lateral:.3f} m sideways; it stopped at the wall '
        'instead of sliding along it')


def test_multiple_planes_form_a_box(chain):
    start = chain.forward(READY)[:3, 3]
    planes = [([0, 0, 1], start[2] - 0.05),      # floor
              ([0, 0, -1], -(start[2] + 0.05)),  # ceiling
              ([0, 1, 0], start[1] - 0.05)]      # side wall
    solver = IKSolver(chain, nominal=READY, planes=planes)
    solver.reset(READY)
    for target in ([0.6, -0.6, -0.6], [0.0, 0.6, 0.6]):
        solver.reset(READY)
        for _ in range(20):
            r = solver.solve(np.array(target), chain.forward(READY)[:3, :3])
        assert not solver.plane_violations(r.q), f'escaped the box toward {target}'


def test_rejects_a_degenerate_plane(chain):
    with pytest.raises(ValueError, match='non-zero'):
        IKSolver(chain, nominal=READY, planes=[([0, 0, 0], 0.0)])


# ---- per-solve smoothness --------------------------------------------------
# The weight is an Opti parameter rather than a baked constant so that ONE
# solve can be retried pulled hard toward the previous answer. That retry is
# what turns a refused IK branch flip into a nearby solution instead of a
# frozen arm: refusing on its own leaves the target unchanged, so the next
# solve rediscovers the branch that was just rejected.

def test_smoothness_defaults_to_the_configured_weight(chain):
    s = IKSolver(chain, nominal=READY, weights=IKWeights(smoothness=0.1))
    pose = chain.forward(READY)
    a = s.solve(pose[:3, 3], pose[:3, :3], q_seed=READY).q
    b = s.solve(pose[:3, 3], pose[:3, :3], q_seed=READY,
                smoothness=0.1).q
    assert np.allclose(a, b, atol=1e-6)


def test_raising_smoothness_pulls_the_answer_toward_the_seed(chain):
    """The mechanism the jump retry relies on."""
    s = IKSolver(chain, nominal=READY, weights=IKWeights(smoothness=0.1))
    # A target the seed does not already satisfy, so the pull is observable.
    pose = chain.forward(READY + np.array([0.35, 0.0, 0.0, 0.0, 0.0, 0.0]))
    loose = s.solve(pose[:3, 3], pose[:3, :3], q_seed=READY, smoothness=0.01).q
    tight = s.solve(pose[:3, 3], pose[:3, :3], q_seed=READY, smoothness=50.0).q
    assert (np.max(np.abs(tight - READY))
            < np.max(np.abs(loose - READY))), 'smoothness did not bind'


def test_smoothness_does_not_leak_between_solves(chain):
    """A one-off retry must not change what every later solve returns."""
    s = IKSolver(chain, nominal=READY, weights=IKWeights(smoothness=0.1))
    pose = chain.forward(READY + np.array([0.3, 0.0, 0.0, 0.0, 0.0, 0.0]))
    before = s.solve(pose[:3, 3], pose[:3, :3], q_seed=READY).q
    s.solve(pose[:3, 3], pose[:3, :3], q_seed=READY, smoothness=80.0)
    after = s.solve(pose[:3, 3], pose[:3, :3], q_seed=READY).q
    assert np.allclose(before, after, atol=1e-6)


def test_a_tight_retry_still_reaches_a_reachable_target(chain):
    """Tightening must not cost tracking on targets that are actually near.

    This is the "without making it sluggish" half: the retry only fires on
    frames that would otherwise have been dropped entirely, and on those it
    must still land close to the commanded pose.
    """
    s = IKSolver(chain, nominal=READY, weights=IKWeights(smoothness=0.1))
    near = READY + np.array([0.05, 0.02, -0.03, 0.0, 0.01, 0.0])
    pose = chain.forward(near)
    q = s.solve(pose[:3, 3], pose[:3, :3], q_seed=READY, smoothness=5.0).q
    reached = chain.forward(q)[:3, 3]
    assert np.linalg.norm(reached - pose[:3, 3]) < 0.01
