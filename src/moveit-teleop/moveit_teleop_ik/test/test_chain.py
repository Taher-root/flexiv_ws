"""Tests for the analytic CasADi FK chain.

The oracle is a second, deliberately different implementation: it composes
transforms with scipy's ``Rotation.from_rotvec`` rather than a hand-written
Rodrigues over CasADi symbols. Agreement between the two is meaningful;
comparing an implementation against itself would not be.
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from moveit_teleop_ik.chain import ChainError, SerialChain

# ---- fixtures --------------------------------------------------------------

TWO_LINK = """<?xml version="1.0"?>
<robot name="two_link">
  <link name="base"/><link name="l1"/><link name="l2"/><link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1.5" upper="1.5" effort="10" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="l1"/><child link="l2"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-2.0" upper="0.5" effort="10" velocity="1"/>
  </joint>
  <joint name="fixed_tip" type="fixed">
    <parent link="l2"/><child link="tip"/>
    <origin xyz="0.3 0 0" rpy="0 0 0"/>
  </joint>
</robot>"""

WITH_PRISMATIC = """<?xml version="1.0"?>
<robot name="prism">
  <link name="base"/><link name="l1"/>
  <joint name="slide" type="prismatic">
    <parent link="base"/><child link="l1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="0.4" effort="10" velocity="1"/>
  </joint>
</robot>"""

CONTINUOUS = """<?xml version="1.0"?>
<robot name="cont">
  <link name="base"/><link name="l1"/>
  <joint name="spin" type="continuous">
    <parent link="base"/><child link="l1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
  </joint>
</robot>"""


def oracle_fk(chain, q):
    """Independent FK: scipy rotations composed with 4x4 matrices."""
    T = np.eye(4)
    index = 0
    for joint in chain._chain:
        L = np.eye(4)
        L[:3, :3] = joint.origin_R
        L[:3, 3] = joint.origin_p
        T = T @ L
        if not joint.actuated:
            continue
        M = np.eye(4)
        if joint.type == 'prismatic':
            M[:3, 3] = joint.axis * q[index]
        else:
            M[:3, :3] = Rotation.from_rotvec(joint.axis * q[index]).as_matrix()
        T = T @ M
        index += 1
    return T


# ---- forward kinematics ----------------------------------------------------

def test_two_link_closed_form():
    """A planar 2R arm has FK anyone can write down; check against it."""
    c = SerialChain(TWO_LINK, 'base', 'tip')
    assert c.joint_names == ['j1', 'j2']
    assert c.nq == 2

    for q1, q2 in [(0.0, 0.0), (0.5, -0.3), (1.2, 0.4)]:
        T = c.forward([q1, q2])
        expected = np.array([
            0.5 * np.cos(q1) + 0.3 * np.cos(q1 + q2),
            0.5 * np.sin(q1) + 0.3 * np.sin(q1 + q2),
            0.1,
        ])
        assert np.allclose(T[:3, 3], expected, atol=1e-12)
        assert np.isclose(Rotation.from_matrix(T[:3, :3]).as_rotvec()[2],
                          q1 + q2, atol=1e-12)


def test_matches_independent_oracle_over_random_configurations():
    c = SerialChain(TWO_LINK, 'base', 'tip')
    rng = np.random.default_rng(0)
    for _ in range(500):
        q = rng.uniform(c.lower, c.upper)
        assert np.allclose(c.forward(q), oracle_fk(c, q), atol=1e-9)


def test_fixed_joints_contribute_but_are_not_actuated():
    c = SerialChain(TWO_LINK, 'base', 'tip')
    assert 'fixed_tip' not in c.joint_names
    # The fixed 0.3 m link still shows up in the tip position.
    assert np.isclose(c.forward([0.0, 0.0])[0, 3], 0.8)


def test_prismatic_joint_translates_along_its_axis():
    c = SerialChain(WITH_PRISMATIC, 'base', 'l1')
    assert np.allclose(c.forward([0.25])[:3, 3], [0, 0, 0.25])
    assert np.allclose(c.forward([0.0])[:3, :3], np.eye(3))


def test_continuous_joint_gets_a_finite_box():
    """The optimiser needs bounds; a continuous joint declares none."""
    c = SerialChain(CONTINUOUS, 'base', 'l1')
    assert c.lower[0] == pytest.approx(-2 * np.pi)
    assert c.upper[0] == pytest.approx(2 * np.pi)


def test_joint_limits_are_read_from_the_urdf():
    c = SerialChain(TWO_LINK, 'base', 'tip')
    assert np.allclose(c.lower, [-1.5, -2.0])
    assert np.allclose(c.upper, [1.5, 0.5])


# ---- derivatives -----------------------------------------------------------

def finite_difference_jacobian(chain, q, eps=1e-7):
    """Numeric geometric Jacobian, used ONLY as a test oracle."""
    T0 = chain.forward(q)
    J = np.zeros((6, chain.nq))
    for i in range(chain.nq):
        dq = np.array(q, float)
        dq[i] += eps
        T1 = chain.forward(dq)
        J[:3, i] = (T1[:3, 3] - T0[:3, 3]) / eps
        J[3:, i] = Rotation.from_matrix(T1[:3, :3] @ T0[:3, :3].T).as_rotvec() / eps
    return J


def test_symbolic_jacobian_matches_finite_difference():
    """The whole point of building FK symbolically is exact derivatives."""
    c = SerialChain(TWO_LINK, 'base', 'tip')
    rng = np.random.default_rng(1)
    for _ in range(50):
        q = rng.uniform(c.lower, c.upper)
        assert np.allclose(c.jacobian_at(q), finite_difference_jacobian(c, q),
                           atol=1e-6)


def test_condition_number_flags_a_singular_configuration():
    """A 2R arm fully extended is singular; folded it is not."""
    c = SerialChain(TWO_LINK, 'base', 'tip')
    assert c.condition_number([0.0, 0.0]) > c.condition_number([0.0, -1.0])


# ---- error handling --------------------------------------------------------

def test_unreachable_tip_is_rejected():
    with pytest.raises(ChainError, match='no path'):
        SerialChain(TWO_LINK, 'l1', 'base')


def test_chain_without_actuated_joints_is_rejected():
    urdf = """<?xml version="1.0"?><robot name="r">
      <link name="a"/><link name="b"/>
      <joint name="f" type="fixed"><parent link="a"/><child link="b"/></joint>
    </robot>"""
    with pytest.raises(ChainError, match='no actuated joints'):
        SerialChain(urdf, 'a', 'b')


# ---- the real robot --------------------------------------------------------

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


@pytest.mark.skipif(piper_urdf() is None, reason='Piper URDF not built')
def test_piper_chain_matches_oracle_and_known_conditioning():
    """End-to-end on the real robot, including the facts measured earlier."""
    c = SerialChain(piper_urdf(), 'base_link', 'tcp_link')
    assert c.joint_names == [f'joint{i}' for i in range(1, 7)]

    rng = np.random.default_rng(2)
    for _ in range(200):
        q = rng.uniform(c.lower, c.upper)
        assert np.allclose(c.forward(q), oracle_fk(c, q), atol=1e-9)
        assert np.allclose(c.jacobian_at(q), finite_difference_jacobian(c, q),
                           atol=1e-5)

    # Measured independently of the solver: the zero pose is singular and
    # ready_pose is not. If these ever invert, something is very wrong.
    ready = [0.0, 1.65, -0.8, 0.0, 0.8, 0.0]
    assert c.condition_number(ready) < 50
    assert c.condition_number(np.zeros(6)) > 1000
    # Mid-range is NOT a safe nominal on this arm -- it is worse than home.
    assert c.condition_number(c.midrange()) > 1000
