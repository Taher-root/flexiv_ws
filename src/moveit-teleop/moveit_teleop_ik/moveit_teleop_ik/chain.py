"""Analytic forward kinematics for a serial chain, as a CasADi expression.

The chain is read from a URDF **string** and the tip pose is composed in closed
form: for each joint, its fixed ``origin`` transform followed by a rotation of
``q_i`` about the joint's own axis (Rodrigues). Nothing is approximated and
nothing is numerically differentiated -- building the expression over
``casadi.SX`` lets CasADi differentiate it symbolically, so the optimiser gets
exact analytic gradients.

Everything robot-specific comes from the URDF. No link or joint name appears
here, so any manipulator MoveIt describes as a ``<chain>`` group works
unchanged.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass

import casadi as ca
import numpy as np
from scipy.spatial.transform import Rotation

#: Bounds used for a ``continuous`` joint, which declares none. The optimiser
#: needs a box, and a full turn either way is generous without being unbounded.
CONTINUOUS_LIMIT = 2.0 * np.pi


class ChainError(ValueError):
    pass


@dataclass
class _Joint:
    name: str
    type: str
    origin_R: np.ndarray      # 3x3, constant
    origin_p: np.ndarray      # 3, constant
    axis: np.ndarray          # 3, unit
    lower: float
    upper: float
    velocity: float           # rad/s or m/s; inf when the URDF declares none
    parent: str
    child: str

    @property
    def actuated(self) -> bool:
        return self.type in ('revolute', 'continuous', 'prismatic')


def _rpy_to_matrix(rpy) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw -> rotation matrix.

    URDF's convention is extrinsic x-y-z, i.e. Rz(yaw) @ Ry(pitch) @ Rx(roll),
    which is what scipy's lowercase 'xyz' gives.
    """
    return Rotation.from_euler('xyz', rpy).as_matrix()


def _rodrigues(axis: np.ndarray, angle):
    """Rotation matrix about a constant unit ``axis`` by a symbolic ``angle``.

    Written out rather than taken from a library because ``angle`` is a CasADi
    symbol: the result has to be an expression CasADi can differentiate.
    """
    kx, ky, kz = float(axis[0]), float(axis[1]), float(axis[2])
    K = ca.SX(3, 3)
    K[0, 1], K[0, 2] = -kz, ky
    K[1, 0], K[1, 2] = kz, -kx
    K[2, 0], K[2, 1] = -ky, kx
    return ca.SX.eye(3) + ca.sin(angle) * K + (1.0 - ca.cos(angle)) * ca.mtimes(K, K)


class SerialChain:
    """Forward kinematics from ``base_link`` to ``tip_link``.

    Parameters
    ----------
    urdf_xml
        The robot description, as a string (not a path) -- it normally comes
        from a running node's ``robot_description`` parameter.
    base_link, tip_link
        Ends of the chain. Read them from the SRDF via
        ``moveit_teleop_core.robot_info.RobotInfo.group_chain``.
    """

    def __init__(self, urdf_xml: str, base_link: str, tip_link: str):
        root = ET.fromstring(urdf_xml)
        self.base_link = base_link
        self.tip_link = tip_link

        by_child = {}
        for element in root.findall('joint'):
            child = element.find('child')
            parent = element.find('parent')
            if child is None or parent is None:
                continue
            origin = element.find('origin')
            xyz = np.zeros(3)
            rpy = np.zeros(3)
            if origin is not None:
                xyz = np.array([float(v) for v in (origin.get('xyz') or '0 0 0').split()])
                rpy = np.array([float(v) for v in (origin.get('rpy') or '0 0 0').split()])

            axis_element = element.find('axis')
            axis = np.array([float(v) for v in axis_element.get('xyz').split()]) \
                if axis_element is not None else np.array([0.0, 0.0, 1.0])
            norm = np.linalg.norm(axis)
            if norm > 0:
                axis = axis / norm

            jtype = element.get('type')
            limit = element.find('limit')
            if jtype == 'continuous':
                lower, upper = -CONTINUOUS_LIMIT, CONTINUOUS_LIMIT
            elif limit is not None:
                lower = float(limit.get('lower', -CONTINUOUS_LIMIT))
                upper = float(limit.get('upper', CONTINUOUS_LIMIT))
            else:
                lower, upper = -CONTINUOUS_LIMIT, CONTINUOUS_LIMIT
            velocity = float(limit.get('velocity')) \
                if limit is not None and limit.get('velocity') else float('inf')

            by_child[child.get('link')] = _Joint(
                name=element.get('name'), type=jtype,
                origin_R=_rpy_to_matrix(rpy), origin_p=xyz, axis=axis,
                lower=lower, upper=upper, velocity=velocity,
                parent=parent.get('link'), child=child.get('link'))

        # Walk from the tip back to the base, then reverse.
        chain, link, seen = [], tip_link, set()
        while link != base_link:
            if link in seen:
                raise ChainError(f'cycle in the URDF at {link!r}')
            seen.add(link)
            if link not in by_child:
                raise ChainError(
                    f'no path from {tip_link!r} back to {base_link!r}: '
                    f'{link!r} has no parent joint')
            joint = by_child[link]
            chain.append(joint)
            link = joint.parent
        chain.reverse()

        self._chain = chain
        self._actuated = [j for j in chain if j.actuated]
        if not self._actuated:
            raise ChainError(
                f'chain {base_link!r} -> {tip_link!r} has no actuated joints')
        self.joint_names = [j.name for j in self._actuated]
        self.nq = len(self._actuated)
        self.lower = np.array([j.lower for j in self._actuated])
        self.upper = np.array([j.upper for j in self._actuated])
        #: Per-joint velocity limits from the URDF. Used to rate-limit the
        #: command stream so we never ask for motion the joint cannot deliver.
        self.velocity_limits = np.array([j.velocity for j in self._actuated])

        self._build()

    # ---- expression construction -----------------------------------------

    def _build(self) -> None:
        q = ca.SX.sym('q', self.nq)
        R = ca.SX.eye(3)
        p = ca.SX.zeros(3)
        index = 0

        for joint in self._chain:
            # Fixed part: the joint's origin, constant for every joint.
            p = p + ca.mtimes(R, ca.SX(joint.origin_p.reshape(3, 1)))
            R = ca.mtimes(R, ca.SX(joint.origin_R))

            if not joint.actuated:
                continue
            if joint.type == 'prismatic':
                p = p + ca.mtimes(R, ca.SX(joint.axis.reshape(3, 1))) * q[index]
            else:
                R = ca.mtimes(R, _rodrigues(joint.axis, q[index]))
            index += 1

        self._q = q
        self._p = p
        self._R = R
        self.fk = ca.Function('fk', [q], [p, R], ['q'], ['p', 'R'])

        # Geometric Jacobian: linear rows are d(p)/dq exactly; angular rows are
        # the joint axes expressed in the base frame, accumulated along the
        # chain. Both come straight from the symbolic expression.
        Jv = ca.jacobian(p, q)
        Jw = self._angular_jacobian(q)
        self.jacobian = ca.Function('J', [q], [ca.vertcat(Jv, Jw)], ['q'], ['J'])

    def _angular_jacobian(self, q):
        R = ca.SX.eye(3)
        columns = []
        index = 0
        for joint in self._chain:
            R = ca.mtimes(R, ca.SX(joint.origin_R))
            if not joint.actuated:
                continue
            if joint.type == 'prismatic':
                columns.append(ca.SX.zeros(3, 1))
            else:
                columns.append(ca.mtimes(R, ca.SX(joint.axis.reshape(3, 1))))
                R = ca.mtimes(R, _rodrigues(joint.axis, q[index]))
            index += 1
        return ca.horzcat(*columns)

    # ---- convenience ------------------------------------------------------

    def forward(self, q) -> np.ndarray:
        """Evaluate FK numerically; returns a 4x4 homogeneous transform."""
        p, R = self.fk(np.asarray(q, float))
        T = np.eye(4)
        T[:3, :3] = np.array(R)
        T[:3, 3] = np.array(p).reshape(3)
        return T

    def jacobian_at(self, q) -> np.ndarray:
        return np.array(self.jacobian(np.asarray(q, float)))

    def condition_number(self, q) -> float:
        """Jacobian condition number. Large means near-singular.

        Used to sanity-check a nominal posture at startup: regularising toward
        an ill-conditioned pose biases the solver into a singularity, which is
        exactly the trap this arm's zero pose sets.
        """
        singular = np.linalg.svd(self.jacobian_at(q), compute_uv=False)
        smallest = singular[-1]
        return float(singular[0] / smallest) if smallest > 1e-12 else float('inf')

    def midrange(self) -> np.ndarray:
        return 0.5 * (self.lower + self.upper)

    def clamp(self, q) -> np.ndarray:
        return np.clip(np.asarray(q, float), self.lower, self.upper)
