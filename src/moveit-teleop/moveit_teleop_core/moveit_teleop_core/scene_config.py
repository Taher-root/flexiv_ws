"""Declare the geometry teleoperation must not drive the arm into.

One file describes both mechanisms, because they are two views of the same
obstacle and keeping them as separate numbers is a trap:

  * a **collision object** is geometry the checker sees. It stops motion rather
    than redirecting it, and it protects the whole ARM, not just the tool.
  * a **virtual wall** is a linear constraint inside the IK solve. Exact, no
    lag, and the tool SLIDES along it instead of stopping dead.

A box may declare a ``wall`` face, and the plane is then derived from that
box's bounds. That is the point of doing it here: with the wall height and the
box height configured independently they drift apart silently, and the symptom
is a tool that halts in mid-air well above the table with nothing visibly
wrong. Derived, they cannot disagree.

Pure parsing and geometry: no ROS, no TF, no message types, so the arithmetic
below is testable on its own.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml


class SceneConfigError(ValueError):
    pass


#: Which face of a box a wall constraint is taken from, as (axis, sign).
#: '+z' means "the tool stays on the +z side of the box's upper z face", which
#: is the table case.
FACES = {
    '+x': (0, +1), '-x': (0, -1),
    '+y': (1, +1), '-y': (1, -1),
    '+z': (2, +1), '-z': (2, -1),
}


@dataclass
class Plane:
    """The half-space ``normal . p >= offset``, in ``frame``.

    Optionally BOUNDED. ``extent`` limits where the half-space applies, as
    ``[a_min, a_max, b_min, b_max]`` along the two axes perpendicular to the
    normal (for a +z wall those are x and y). Outside it the wall simply is not
    there, so the tool can drop past the edge of a table instead of being held
    at table height across the whole workspace.

    **Bounded walls are off by default, and the reason is measured.** A wall
    that switches off is a non-convex region approximated by a switch, and a
    vanishing constraint moves the optimum discontinuously: sweeping the target
    across the footprint edge of this arm's table steps the tool 218 mm in a
    single solve, against ~5 mm either side of it. The retargeter cannot absorb
    that -- it re-anchors on discontinuities it causes, but the solver drops the
    constraint on its own.

    So an unbounded wall is the default even though it models a table you
    cannot reach around. Its failure mode is a wall that extends too far, which
    is inconvenient; the bounded one's is a 218 mm lurch, which is not.
    Smoothing the switch would mean fading the constraint with distance past
    the edge, i.e. slowing near an obstacle -- explicitly not wanted here.

    Only axis-aligned normals can carry an extent. For a tilted plane "the two
    perpendicular axes" is a choice rather than a fact, and guessing it would
    silently bound the wall along axes the config never mentioned.
    """

    name: str
    frame: str
    normal: np.ndarray
    offset: float
    extent: np.ndarray = None      # [a_min, a_max, b_min, b_max] or None

    @property
    def axes(self) -> tuple:
        """The two coordinate indices the extent is measured along."""
        axis = int(np.argmax(np.abs(self.normal)))
        return tuple(i for i in range(3) if i != axis)

    def applies_at(self, point) -> bool:
        """Is *point* inside this wall's footprint?"""
        if self.extent is None:
            return True
        i, j = self.axes
        p = np.asarray(point, float)
        return bool(self.extent[0] <= p[i] <= self.extent[1]
                    and self.extent[2] <= p[j] <= self.extent[3])

    def transformed(self, rotation, translation) -> 'Plane':
        """Re-express this plane in another frame.

        Given ``p_target = R @ p_here + t``, a point satisfies the constraint
        when ``n . p_here >= d``. Substituting ``p_here = R^T (p_target - t)``
        gives ``(R n) . p_target >= d + (R n) . t`` -- so the normal rotates and
        the offset picks up the projection of the translation onto it.
        """
        rotation = np.asarray(rotation, float)
        translation = np.asarray(translation, float)
        rotated = rotation @ self.normal

        extent = self.extent
        if extent is not None:
            if not np.allclose(rotation, np.eye(3), atol=1e-6):
                # A rotated rectangle is no longer axis-aligned, so it cannot
                # be stored as four numbers. Refusing beats quietly bounding
                # the wall along the wrong axes.
                raise SceneConfigError(
                    f"wall '{self.name}' has an extent and its frame is "
                    'rotated relative to the planning frame; bounded walls '
                    'only support frames that differ by a translation')
            i, j = self.axes
            extent = np.array([extent[0] + translation[i],
                               extent[1] + translation[i],
                               extent[2] + translation[j],
                               extent[3] + translation[j]])

        return Plane(name=self.name, frame='', normal=rotated,
                     offset=self.offset + float(rotated @ translation),
                     extent=extent)

@dataclass
class Box:
    """An axis-aligned box, given by its bounds in ``frame``."""

    name: str
    frame: str
    bounds: np.ndarray            # [xmin, xmax, ymin, ymax, zmin, zmax]
    collision: bool = True
    wall: str = ''                # '' | one of FACES
    wall_clearance: float = 0.0   # how far off the face the wall sits
    wall_margin: float = 0.0      # extra footprint beyond the box sides
    wall_bounded: bool = False    # True -> only over the box footprint

    @property
    def size(self) -> np.ndarray:
        lo, hi = self.bounds[0::2], self.bounds[1::2]
        return hi - lo

    @property
    def center(self) -> np.ndarray:
        lo, hi = self.bounds[0::2], self.bounds[1::2]
        return (lo + hi) / 2.0

    def wall_plane(self) -> Plane:
        """The half-space the tool is kept in, taken from ``wall``'s face.

        Derived from the SAME bounds the collision box uses, so the surface the
        tool slides on is the surface the checker stops the arm at.
        """
        if not self.wall:
            raise SceneConfigError(f'box {self.name!r} declares no wall face')
        axis, sign = FACES[self.wall]
        normal = np.zeros(3)
        normal[axis] = sign
        # +z -> the face at zmax, and the tool stays above it.
        face = self.bounds[2 * axis + (1 if sign > 0 else 0)]
        plane = Plane(name=f'{self.name}:{self.wall}', frame=self.frame,
                      normal=normal,
                      offset=sign * face + float(self.wall_clearance))
        if self.wall_bounded:
            # The footprint IS the box's own footprint. Derived, like the
            # offset, so a table you can reach around behaves like one and the
            # two can never disagree about where the surface ends.
            i, j = plane.axes
            m = float(self.wall_margin)
            plane.extent = np.array([
                self.bounds[2 * i] - m, self.bounds[2 * i + 1] + m,
                self.bounds[2 * j] - m, self.bounds[2 * j + 1] + m])
        return plane


@dataclass
class SceneConfig:
    boxes: list = field(default_factory=list)
    planes: list = field(default_factory=list)

    @classmethod
    def from_file(cls, path) -> 'SceneConfig':
        text = Path(path).expanduser().read_text()
        raw = yaml.safe_load(text) or {}
        if 'planning_scene' not in raw:
            raise SceneConfigError(
                f'{path}: expected a top-level "planning_scene:" key')
        return cls.from_dict(raw['planning_scene'])

    @classmethod
    def from_dict(cls, raw: dict) -> 'SceneConfig':
        default_frame = raw.get('default_frame', '') or ''

        boxes = []
        for entry in raw.get('boxes') or []:
            name = entry.get('name')
            if not name:
                raise SceneConfigError(f'box entry has no name: {entry}')
            bounds = entry.get('bounds')
            if bounds is None or len(bounds) != 6:
                raise SceneConfigError(
                    f'box {name!r} needs bounds '
                    '[xmin, xmax, ymin, ymax, zmin, zmax]')
            bounds = np.asarray([float(v) for v in bounds])
            for axis, label in enumerate('xyz'):
                lo, hi = bounds[2 * axis], bounds[2 * axis + 1]
                if hi <= lo:
                    raise SceneConfigError(
                        f'box {name!r}: {label}max ({hi}) must exceed '
                        f'{label}min ({lo})')
            wall = str(entry.get('wall', '') or '')
            if wall and wall not in FACES:
                raise SceneConfigError(
                    f'box {name!r}: wall is {wall!r}; expected one of '
                    f'{sorted(FACES)}')
            boxes.append(Box(
                name=name,
                frame=str(entry.get('frame', default_frame) or default_frame),
                bounds=bounds,
                collision=bool(entry.get('collision', True)),
                wall=wall,
                wall_clearance=float(entry.get('wall_clearance', 0.0)),
                wall_margin=float(entry.get('wall_margin', 0.0)),
                wall_bounded=bool(entry.get('wall_bounded', False)),
            ))

        planes = []
        for entry in raw.get('planes') or []:
            name = entry.get('name')
            if not name:
                raise SceneConfigError(f'plane entry has no name: {entry}')
            normal = np.asarray([float(v) for v in entry.get('normal', [])])
            if normal.shape != (3,) or not np.any(normal):
                raise SceneConfigError(
                    f'plane {name!r} needs a non-zero 3-vector normal')
            unit = normal / np.linalg.norm(normal)
            plane = Plane(
                name=name,
                frame=str(entry.get('frame', default_frame) or default_frame),
                normal=unit,
                offset=float(entry.get('offset', 0.0)),
            )
            extent = entry.get('extent')
            if extent is not None:
                if len(extent) != 4:
                    raise SceneConfigError(
                        f'plane {name!r}: extent is [a_min, a_max, b_min, '
                        'b_max] along the two axes perpendicular to the '
                        'normal')
                axis = int(np.argmax(np.abs(unit)))
                if abs(abs(unit[axis]) - 1.0) > 1e-6:
                    raise SceneConfigError(
                        f'plane {name!r} has an extent but its normal '
                        f'{unit.round(3).tolist()} is not axis-aligned; the '
                        'two perpendicular axes would be a guess')
                plane.extent = np.asarray([float(v) for v in extent])
            planes.append(plane)

        names = [b.name for b in boxes] + [p.name for p in planes]
        duplicates = {n for n in names if names.count(n) > 1}
        if duplicates:
            # Two objects sharing an id means the second silently REPLACES the
            # first in the planning scene.
            raise SceneConfigError(
                f'duplicate scene object names: {sorted(duplicates)}')
        return cls(boxes=boxes, planes=planes)

    def all_planes(self) -> list:
        """Every wall: those declared directly, plus those derived from boxes."""
        return list(self.planes) + [b.wall_plane() for b in self.boxes if b.wall]

    def collision_boxes(self) -> list:
        return [b for b in self.boxes if b.collision]


def _half_space(plane) -> tuple:
    """Accept either a Plane or a bare ``(normal, offset)`` pair."""
    if isinstance(plane, Plane):
        return plane.normal, plane.offset
    return plane[0], plane[1]


def _applies(plane, point) -> bool:
    return plane.applies_at(point) if isinstance(plane, Plane) else True


def project(point, planes, iterations: int = 8):
    """Move *point* to the nearest place satisfying every plane.

    For one plane this is exact: a point violating ``n . p >= d`` is pushed
    along ``n`` by exactly the shortfall, which lands it ON the surface and
    leaves the two tangential components untouched. That tangential invariance
    is the whole point -- it is what lets the tool SLIDE along a table instead
    of stopping dead when the operator's hand goes below it.

    For several planes the passes are applied in turn and repeated. That
    converges to the intersection for the near-orthogonal arrangements this is
    for (a table and a back wall); it is not Dykstra's algorithm and will not
    find the exact nearest point of a narrow wedge. Returned unchanged when
    already feasible, so the common case costs nothing.
    """
    p = np.asarray(point, float).copy()
    if not planes:
        return p
    for _ in range(max(1, iterations)):
        moved = False
        for plane in planes:
            normal, offset = _half_space(plane)
            if not _applies(plane, p):
                # Off the end of this wall: it is simply not there, so the
                # tool may pass the level of a table it has reached around.
                continue
            shortfall = float(offset) - float(np.dot(normal, p))
            if shortfall > 0.0:
                # normal is unit length, so this is the exact distance.
                p = p + shortfall * np.asarray(normal, float)
                moved = True
        if not moved:
            break
    return p


def violations(point, planes, tolerance: float = 1e-9) -> list:
    """Which planes *point* is on the wrong side of, worst first."""
    p = np.asarray(point, float)
    out = []
    for index, plane in enumerate(planes):
        normal, offset = _half_space(plane)
        if not _applies(plane, p):
            continue
        depth = float(offset) - float(np.dot(normal, p))
        if depth > tolerance:
            out.append((index, depth))
    out.sort(key=lambda pair: pair[1], reverse=True)
    return out


def resolve_planes(node, path, planning_frame, timeout_sec: float = 10.0):
    """Load a scene file and express every wall in *planning_frame*.

    Shared by the retargeter and the IK solver so the two cannot end up with
    different ideas of where a wall is -- which is the same class of bug as
    configuring the wall height separately from the box.

    Returns a list of :class:`Plane`, each already in *planning_frame*,
    so callers keep the footprint as well as the half-space.
    """
    if not path:
        return []

    import rclpy
    from scipy.spatial.transform import Rotation
    from tf2_ros import Buffer, TransformListener

    scene = SceneConfig.from_file(path)
    walls = scene.all_planes()
    if not walls:
        return []

    buffer = Buffer()
    # Held on the node so the listener is not garbage collected the moment
    # this function returns, which would silently stop TF arriving.
    node._scene_tf = (buffer, TransformListener(buffer, node))

    import time
    resolved = []
    for wall in walls:
        frame = wall.frame or planning_frame
        if frame == planning_frame:
            resolved.append(wall)
            continue
        deadline = time.time() + timeout_sec
        transform = None
        while time.time() < deadline and rclpy.ok():
            try:
                transform = buffer.lookup_transform(
                    planning_frame, frame, rclpy.time.Time()).transform
                break
            except Exception:                       # noqa: BLE001
                rclpy.spin_once(node, timeout_sec=0.1)
        if transform is None:
            # Dropping a wall would leave the tool free to enter the very
            # region the file exists to protect, so this is fatal.
            raise RuntimeError(
                f"wall '{wall.name}' is declared in frame {frame!r}, which "
                f'never appeared in TF relative to {planning_frame!r}')
        rotation = Rotation.from_quat([
            transform.rotation.x, transform.rotation.y,
            transform.rotation.z, transform.rotation.w]).as_matrix()
        translation = [transform.translation.x, transform.translation.y,
                       transform.translation.z]
        moved = wall.transformed(rotation, translation)
        moved.name, moved.frame = wall.name, planning_frame
        resolved.append(moved)
    return resolved
