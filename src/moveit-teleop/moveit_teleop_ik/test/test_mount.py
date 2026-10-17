"""An arm whose base is not the planning frame.

Targets arrive in the planning frame (the URDF root); the chain's FK starts at
the group's base link. On an arm mounted on a waist the two differ by a
transform that moves, and solving without it sends the arm after a target
displaced by the whole mount -- it reaches for it and straightens out.
"""

import numpy as np
import pytest

from moveit_teleop_ik.chain import ChainError, Mount, SerialChain
from moveit_teleop_ik.ik_solver import IKSolver, IKWeights

URDF = """
<robot name="mounted">
  <link name="world"/><link name="waist"/><link name="arm_base"/>
  <link name="l1"/><link name="l2"/><link name="l3"/><link name="l4"/>
  <link name="tip"/>
  <joint name="waist_yaw" type="revolute">
    <parent link="world"/><child link="waist"/>
    <origin xyz="0 0 0.5"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" velocity="1" effort="1"/>
  </joint>
  <joint name="shoulder_mount" type="fixed">
    <parent link="waist"/><child link="arm_base"/>
    <origin xyz="0 0.2 0.6" rpy="1.5708 0 0"/>
  </joint>
  <joint name="j1" type="revolute">
    <parent link="arm_base"/><child link="l1"/>
    <axis xyz="0 0 1"/><limit lower="-3" upper="3" velocity="2" effort="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="l1"/><child link="l2"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" velocity="2" effort="1"/>
  </joint>
  <joint name="j3" type="revolute">
    <parent link="l2"/><child link="l3"/>
    <origin xyz="0 0 0.3"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" velocity="2" effort="1"/>
  </joint>
  <joint name="j4" type="revolute">
    <parent link="l3"/><child link="l4"/>
    <origin xyz="0 0 0.3"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" velocity="2" effort="1"/>
  </joint>
  <joint name="j5" type="revolute">
    <parent link="l4"/><child link="tip"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" velocity="2" effort="1"/>
  </joint>
</robot>
"""

BENT = np.array([0.2, 0.6, 1.2, 0.1, 0.5])


@pytest.fixture(scope='module')
def arm():
    return SerialChain(URDF, 'arm_base', 'tip')


@pytest.fixture(scope='module')
def mount():
    return Mount(URDF, 'world', 'arm_base')


def test_mount_matches_full_chain_fk(mount):
    reference = SerialChain(URDF, 'world', 'l1')   # includes j1 at 0
    assert mount.joint_names == ['waist_yaw']
    for yaw in (0.0, 0.7, -1.2):
        expected = reference.forward([yaw, 0.0])
        np.testing.assert_allclose(mount.transform({'waist_yaw': yaw}), expected,
                                   atol=1e-9)


def test_fixed_only_and_identity_mounts():
    fixed = Mount(URDF, 'waist', 'arm_base')
    assert fixed.joint_names == []
    np.testing.assert_allclose(fixed.transform({})[:3, 3], [0, 0.2, 0.6])
    np.testing.assert_allclose(Mount(URDF, 'world', 'world').transform({}), np.eye(4))
    with pytest.raises(ChainError):
        Mount(URDF, 'tip', 'world')


@pytest.mark.parametrize('yaw', [0.0, 0.8])
def test_holds_a_planning_frame_target_at_the_current_pose(arm, mount, yaw):
    """Clutch engaged, hand still: the arm must not move."""
    solver = IKSolver(arm, nominal=BENT, weights=IKWeights(50, 1, 0.02, 0.1))
    M = mount.transform({'waist_yaw': yaw})
    solver.set_mount(M)
    solver.reset(BENT)
    T = M @ arm.forward(BENT)
    result = solver.solve(T[:3, 3], T[:3, :3])
    assert result.position_error < 1e-3
    assert np.max(np.abs(result.q - BENT)) < 1e-2


def test_without_the_mount_the_same_target_pulls_the_arm_away(arm, mount):
    """The bug: a planning-frame target read in the chain-base frame."""
    solver = IKSolver(arm, nominal=BENT, weights=IKWeights(50, 1, 0.02, 0.1))
    solver.reset(BENT)
    T = mount.transform({'waist_yaw': 0.0}) @ arm.forward(BENT)
    result = solver.solve(T[:3, 3], T[:3, :3])
    assert np.max(np.abs(result.q - BENT)) > 0.2


def test_walls_are_in_the_planning_frame(arm, mount):
    M = mount.transform({'waist_yaw': 0.0})
    tool_z = (M @ arm.forward(BENT))[2, 3]
    solver = IKSolver(arm, nominal=BENT, planes=[([0, 0, 1], tool_z + 0.05)])
    solver.set_mount(M)
    solver.reset(BENT)
    T = M @ arm.forward(BENT)
    result = solver.solve(T[:3, 3], T[:3, :3])
    assert solver.plane_violations(result.q) == []
    assert (M @ arm.forward(result.q))[2, 3] >= tool_z + 0.05 - 1e-4
