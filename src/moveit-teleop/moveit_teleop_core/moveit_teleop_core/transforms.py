"""Rigid-transform helpers for retargeting.

Three operations here are the ones teleoperation code habitually gets wrong,
so each is implemented once, named, and tested:

1. **Change of basis.** Re-expressing a device-frame *transform* in ROS axes is
   a similarity transform ``C T C^-1``, not a permutation of the translation
   components. Permuting translation alone leaves the rotation expressed in the
   old basis; the result looks almost right and drifts wrong as soon as the
   operator rotates their hand.

2. **Rotation scaling.** Halving a rotation means halving its angle about its
   axis, which requires going through axis-angle. Scaling quaternion components
   directly is not a rotation at all.

3. **Rotation smoothing.** Averaging quaternion components is only valid for
   very small differences and breaks across the double-cover sign flip. Use
   slerp.

Uses scipy's Rotation rather than hand-rolled quaternion algebra deliberately:
python3-scipy is a system package here and already a dependency of the vendored
arm driver, and this is not the place to be clever.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

#: Rotations mapping a device's axes into ROS axes (REP-103: x forward,
#: y left, z up). Each maps a vector expressed in the device frame to the same
#: physical vector expressed in ROS axes.
CONVENTIONS = {
    # The device already reports in ROS axes. This is what the fake device and
    # a leader arm use.
    'ros': np.eye(3),

    # WebXR / OpenGL: x right, y up, -z forward. Right-handed, so this is a
    # proper rotation and the similarity transform below is valid as-is.
    # This is the convention Vuer-based headset apps (e.g. xr_teleoperate)
    # report in.
    'webxr': np.array([
        [0.0, 0.0, -1.0],   # ROS x (forward) = -WebXR z
        [-1.0, 0.0, 0.0],   # ROS y (left)    = -WebXR x
        [0.0, 1.0, 0.0],    # ROS z (up)      = +WebXR y
    ]),
}


def convention_matrix(name_or_matrix) -> np.ndarray:
    """Resolve a convention name (or an explicit 3x3) to a rotation matrix."""
    if isinstance(name_or_matrix, str):
        key = name_or_matrix.lower()
        if key == 'unity':
            # Unity is LEFT-handed. Converting to a right-handed frame is a
            # reflection (determinant -1), not a rotation, so the similarity
            # transform used here does not apply and the quaternion needs its
            # handedness flipped too. Refusing is better than shipping math
            # that looks plausible and mirrors the operator's wrist.
            raise NotImplementedError(
                "the 'unity' convention is left-handed and needs a reflection, "
                'not a rotation; convert to a right-handed frame in the device '
                'driver and report it as another convention')
        if key not in CONVENTIONS:
            raise ValueError(
                f'unknown convention {name_or_matrix!r}; '
                f'known: {sorted(CONVENTIONS)}')
        return CONVENTIONS[key]

    matrix = np.asarray(name_or_matrix, dtype=float).reshape(3, 3)
    if not np.isclose(np.linalg.det(matrix), 1.0, atol=1e-6):
        raise ValueError(
            f'convention matrix must be a proper rotation (det=+1), '
            f'got det={np.linalg.det(matrix):.6f}')
    return matrix


def make_transform(position, quaternion_xyzw) -> np.ndarray:
    """Build a 4x4 homogeneous transform from a position and (x, y, z, w)."""
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(np.asarray(quaternion_xyzw, float)).as_matrix()
    transform[:3, 3] = np.asarray(position, float)
    return transform


def split_transform(transform: np.ndarray) -> tuple:
    """Inverse of :func:`make_transform`: return ``(position, xyzw)``."""
    position = np.asarray(transform)[:3, 3].copy()
    quaternion = Rotation.from_matrix(np.asarray(transform)[:3, :3]).as_quat()
    return position, quaternion


def invert_transform(transform: np.ndarray) -> np.ndarray:
    """Invert a rigid transform without a general matrix inverse."""
    rotation = np.asarray(transform)[:3, :3]
    inverse = np.eye(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ np.asarray(transform)[:3, 3]
    return inverse


def change_basis(transform: np.ndarray, convention: np.ndarray) -> np.ndarray:
    """Re-express a transform in another orthonormal basis: ``C T C^-1``.

    ``convention`` maps device-frame vectors to ROS-frame vectors. Applying it
    on both sides is what converts the *rotation* as well as the translation;
    see the module docstring for why the one-sided version is a trap.
    """
    basis = np.eye(4)
    basis[:3, :3] = convention
    return basis @ np.asarray(transform) @ invert_transform(basis)


def heading_rotation(rotation: np.ndarray) -> np.ndarray:
    """The purely-horizontal part of *rotation*: which way it faces.

    Returns Rz(yaw), the rotation about ROS +z (up) that *rotation* contains.
    Used to answer "which way is the operator facing", so that a delta measured
    in the room can be re-expressed as forward/left/up RELATIVE TO THEM.

    Without this, teleoperation is referenced to the play space: push the
    controller away from you and the arm reaches forward, then turn your body
    180 degrees, push away from you again, and the arm reaches BACKWARD --
    because "away from you" is now the opposite direction in the room, and the
    room is what the delta was measured in.

    Computed as the swing-twist decomposition about +z rather than as an Euler
    yaw. Euler yaw comes from the frame's x axis, which goes ill-conditioned
    exactly when the controller points up or down -- and pointing a controller
    down is a completely ordinary way to hold one. The twist stays well behaved
    there and degenerates only at a 180-degree rotation about a HORIZONTAL
    axis, which takes deliberate effort to reach and is handled below.
    """
    quaternion = Rotation.from_matrix(np.asarray(rotation, float)).as_quat()
    x, y, z, w = quaternion
    norm = np.hypot(w, z)
    if norm < 1e-8:
        # Exactly 180 degrees about a horizontal axis: the twist about z is
        # genuinely undefined, not merely hard to compute. Identity keeps the
        # mapping continuous rather than snapping it to an arbitrary heading.
        return np.eye(3)
    return Rotation.from_quat([0.0, 0.0, z / norm, w / norm]).as_matrix()


def scale_transform(transform: np.ndarray, translation_scale: float,
                    rotation_scale: float) -> np.ndarray:
    """Scale a delta transform's translation and rotation independently.

    Rotation is scaled through axis-angle: the axis is preserved and the angle
    multiplied, which is the only interpretation under which "half the
    rotation" means anything.
    """
    rotation = Rotation.from_matrix(np.asarray(transform)[:3, :3])
    rotation_vector = rotation.as_rotvec()          # axis * angle
    scaled = Rotation.from_rotvec(rotation_vector * float(rotation_scale))

    result = np.eye(4)
    result[:3, :3] = scaled.as_matrix()
    result[:3, 3] = np.asarray(transform)[:3, 3] * float(translation_scale)
    return result


def scale_rotation(matrix: np.ndarray, factor: float) -> np.ndarray:
    """Scale a rotation's ANGLE about its own axis, keeping the axis."""
    rotvec = Rotation.from_matrix(np.asarray(matrix)).as_rotvec()
    return Rotation.from_rotvec(rotvec * float(factor)).as_matrix()


class PoseSmoother:
    """Exponential smoothing of a pose stream.

    Translation is smoothed componentwise; rotation is smoothed by slerp
    towards the new sample, which stays correct through the quaternion double
    cover where a componentwise average does not.

    ``alpha`` is the weight of the NEW sample: 1.0 disables smoothing, small
    values lag more.
    """

    def __init__(self, alpha: float = 1.0):
        if not 0.0 < alpha <= 1.0:
            raise ValueError(f'alpha must be in (0, 1], got {alpha}')
        self.alpha = float(alpha)
        self._position = None
        self._rotation = None

    def reset(self) -> None:
        """Forget history. Call on clutch engage so the operator does not drag
        the previous engagement's pose into the new one."""
        self._position = None
        self._rotation = None

    def __call__(self, position, quaternion_xyzw) -> tuple:
        position = np.asarray(position, float)
        rotation = Rotation.from_quat(np.asarray(quaternion_xyzw, float))

        if self._position is None or self.alpha >= 1.0:
            self._position, self._rotation = position, rotation
            return position, rotation.as_quat()

        self._position = (1.0 - self.alpha) * self._position + self.alpha * position
        slerp = Slerp([0.0, 1.0], Rotation.concatenate([self._rotation, rotation]))
        self._rotation = slerp([self.alpha])[0]
        return self._position.copy(), self._rotation.as_quat()
