"""Integrate Cartesian twists into RDK tcp_pose [x,y,z,qw,qx,qy,qz]."""

from __future__ import annotations

from typing import List

import numpy as np

_EPS = 1e-9


def _quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n < _EPS:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def _quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def rotvec_to_quat(rotvec: np.ndarray) -> np.ndarray:
    """Rotation vector (rad) -> unit quaternion wxyz."""
    angle = float(np.linalg.norm(rotvec))
    if angle < _EPS:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = rotvec / angle
    half = 0.5 * angle
    return np.array([np.cos(half), *(axis * np.sin(half))])


def integrate_tcp_pose(
    pose: List[float], linear: List[float], angular: List[float], dt: float
) -> List[float]:
    """Advance world-frame TCP pose by twist * dt (twist in same frame as pose)."""
    if dt <= _EPS:
        return list(pose)
    p = np.asarray(pose, dtype=float)
    lin = np.asarray(linear, dtype=float)
    ang = np.asarray(angular, dtype=float)
    p[0:3] += lin * dt
    q = _quat_normalize(p[3:7])
    dq = rotvec_to_quat(ang * dt)
    p[3:7] = _quat_normalize(_quat_multiply(dq, q))
    return [float(x) for x in p]
