"""Tests for the three operations teleop code habitually gets wrong."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from moveit_teleop_core.transforms import (
    CONVENTIONS, PoseSmoother, change_basis, convention_matrix,
    invert_transform, make_transform, scale_transform, split_transform,
)


def test_make_split_roundtrip():
    pos = [0.1, -0.2, 0.3]
    quat = Rotation.from_euler('xyz', [0.3, -0.7, 1.1]).as_quat()
    p, q = split_transform(make_transform(pos, quat))
    assert np.allclose(p, pos)
    # Quaternions double-cover: q and -q are the same rotation.
    assert np.allclose(Rotation.from_quat(q).as_matrix(),
                       Rotation.from_quat(quat).as_matrix())


def test_invert_transform_matches_numpy():
    T = make_transform([1.0, 2.0, 3.0],
                       Rotation.from_euler('xyz', [0.2, 0.4, -0.6]).as_quat())
    assert np.allclose(invert_transform(T), np.linalg.inv(T))
    assert np.allclose(invert_transform(T) @ T, np.eye(4), atol=1e-12)


def test_all_conventions_are_proper_rotations():
    """A reflection here would mirror the operator's wrist."""
    for name, matrix in CONVENTIONS.items():
        assert np.isclose(np.linalg.det(matrix), 1.0), f'{name} is not proper'
        assert np.allclose(matrix @ matrix.T, np.eye(3)), f'{name} not orthonormal'


def test_webxr_axes_map_as_documented():
    """WebXR is x-right, y-up, -z-forward; ROS is x-forward, y-left, z-up."""
    C = CONVENTIONS['webxr']
    assert np.allclose(C @ [0, 0, -1], [1, 0, 0])   # forward -> +x
    assert np.allclose(C @ [0, 1, 0], [0, 0, 1])    # up      -> +z
    assert np.allclose(C @ [1, 0, 0], [0, -1, 0])   # right   -> -y (ROS y is left)


def test_unity_is_refused_rather_than_guessed():
    with pytest.raises(NotImplementedError, match='left-handed'):
        convention_matrix('unity')


def test_convention_matrix_rejects_reflections():
    with pytest.raises(ValueError, match='proper rotation'):
        convention_matrix(np.diag([1.0, 1.0, -1.0]))


def test_change_basis_transforms_rotation_not_just_translation():
    """The one-sided trap: C@T changes translation but leaves rotation wrong.

    A pure device-frame rotation about WebXR's up axis (+y) must come out as a
    ROS rotation about +z. Permuting translation alone cannot do that.
    """
    C = CONVENTIONS['webxr']
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec([0, 0.5, 0]).as_matrix()   # about WebXR up

    correct = change_basis(T, C)
    axis = Rotation.from_matrix(correct[:3, :3]).as_rotvec()
    assert np.allclose(axis, [0, 0, 0.5], atol=1e-9), 'up-axis rotation not mapped to +z'

    naive = np.eye(4)
    naive[:3, :3] = C @ T[:3, :3]
    assert not np.allclose(naive[:3, :3], correct[:3, :3]), \
        'test is vacuous: the naive one-sided form happened to agree'


def test_change_basis_preserves_rotation_angle():
    """A change of basis is a rigid re-expression; magnitudes cannot change."""
    C = CONVENTIONS['webxr']
    for vec in ([0.4, 0, 0], [0, 0.9, 0], [0.1, -0.2, 0.3]):
        T = np.eye(4)
        T[:3, :3] = Rotation.from_rotvec(vec).as_matrix()
        T[:3, 3] = [0.1, 0.2, 0.3]
        out = change_basis(T, C)
        assert np.isclose(np.linalg.norm(Rotation.from_matrix(out[:3, :3]).as_rotvec()),
                          np.linalg.norm(vec))
        assert np.isclose(np.linalg.norm(out[:3, 3]), np.linalg.norm(T[:3, 3]))


def test_scale_transform_halves_the_angle_not_the_components():
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec([0.0, 0.0, 1.0]).as_matrix()   # 1 rad about z
    T[:3, 3] = [0.2, 0.0, 0.0]

    out = scale_transform(T, 0.5, 0.5)
    assert np.allclose(out[:3, 3], [0.1, 0.0, 0.0])
    assert np.allclose(Rotation.from_matrix(out[:3, :3]).as_rotvec(), [0, 0, 0.5])
    # Still a valid rotation -- naive component scaling would not be.
    assert np.allclose(out[:3, :3] @ out[:3, :3].T, np.eye(3), atol=1e-12)


def test_scale_transform_preserves_axis():
    axis = np.array([1.0, -2.0, 0.5]); axis /= np.linalg.norm(axis)
    T = np.eye(4); T[:3, :3] = Rotation.from_rotvec(axis * 0.8).as_matrix()
    out = scale_transform(T, 1.0, 0.25)
    got = Rotation.from_matrix(out[:3, :3]).as_rotvec()
    assert np.isclose(np.linalg.norm(got), 0.2)
    assert np.allclose(got / np.linalg.norm(got), axis)


def test_identity_scaling_is_identity():
    T = make_transform([0.1, 0.2, 0.3],
                       Rotation.from_euler('xyz', [0.1, 0.2, 0.3]).as_quat())
    assert np.allclose(scale_transform(T, 1.0, 1.0), T)


def test_smoother_alpha_one_is_passthrough():
    s = PoseSmoother(1.0)
    for _ in range(3):
        p, q = s([1.0, 2.0, 3.0], [0, 0, 0, 1])
        assert np.allclose(p, [1.0, 2.0, 3.0])


def test_smoother_lags_then_converges():
    s = PoseSmoother(0.5)
    s([0.0, 0.0, 0.0], [0, 0, 0, 1])
    p, _ = s([1.0, 0.0, 0.0], [0, 0, 0, 1])
    assert np.isclose(p[0], 0.5), 'first step should be half way'
    for _ in range(40):
        p, _ = s([1.0, 0.0, 0.0], [0, 0, 0, 1])
    assert np.isclose(p[0], 1.0, atol=1e-6), 'should converge on a held input'


def test_smoother_handles_quaternion_double_cover():
    """q and -q are the same rotation; componentwise averaging would collapse."""
    s = PoseSmoother(0.5)
    q = Rotation.from_euler('z', 0.4).as_quat()
    s([0, 0, 0], q)
    _, out = s([0, 0, 0], -q)     # identical rotation, opposite sign
    angle = (Rotation.from_quat(out) * Rotation.from_quat(q).inv()).magnitude()
    assert angle < 1e-6, f'smoothing across the double cover rotated by {angle}'


def test_smoother_rejects_bad_alpha():
    for bad in (0.0, -0.1, 1.5):
        with pytest.raises(ValueError):
            PoseSmoother(bad)
