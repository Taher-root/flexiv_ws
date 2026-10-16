"""The retargeting plugin seam.

A retargeter answers one question: given the operator's device pose and the
robot's current end-effector pose, where should the end effector go?

Keeping this behind an interface is what lets a 2-finger gripper on a 6-DoF
arm, a dexterous hand, and a bimanual rig be three strategies over one
pipeline instead of three forks of the same node.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

#: setuptools entry point group. Third parties register a retargeter from
#: their own package; nothing in this package needs editing.
ENTRY_POINT_GROUP = 'moveit_teleop.retargeters'


@dataclass
class RetargetInput:
    """One normalized device sample, plus the robot state it is judged against."""

    #: Device position in the device's own frame.
    device_position: np.ndarray
    #: Device orientation (x, y, z, w) in the device's own frame.
    device_orientation: np.ndarray
    #: Level-triggered engage.
    clutch: bool
    #: False when device tracking is lost.
    pose_valid: bool
    #: Normalized gripper command in [0, 1].
    gripper: float
    #: Current EE position in the planning frame.
    ee_position: np.ndarray
    #: Current EE orientation (x, y, z, w) in the planning frame.
    ee_orientation: np.ndarray


@dataclass
class RetargetOutput:
    """A commanded end-effector pose in the planning frame.

    ``active`` is False when the retargeter is declining to command anything
    (declutched, tracking lost, no anchor yet). The caller must treat that as
    "do not send", NOT as "send the last pose": in POSE mode servo keeps
    driving toward whatever target it last accepted, so silence is not a stop.
    """

    position: np.ndarray
    orientation: np.ndarray
    gripper: float
    active: bool
    reason: str = ''


class Retargeter(ABC):
    """Maps normalized device input to end-effector pose targets."""

    def __init__(self, config: dict, robot_info=None):
        self.config = dict(config or {})
        self.robot_info = robot_info

    @abstractmethod
    def update(self, sample: RetargetInput) -> RetargetOutput:
        """Return the pose to command for this sample."""

    @abstractmethod
    def reset(self) -> None:
        """Drop all accumulated state.

        Called on disengage, on fault, and whenever something else has moved
        the arm (a planned motion, a move to the ready posture). Failing to
        reset after an external motion is what produces a lurch on the next
        engage.
        """


def load_retargeter(name: str, config: dict, robot_info=None) -> Retargeter:
    """Instantiate a retargeter registered under :data:`ENTRY_POINT_GROUP`."""
    from importlib.metadata import entry_points

    matches = [ep for ep in entry_points(group=ENTRY_POINT_GROUP) if ep.name == name]
    if not matches:
        available = sorted(ep.name for ep in entry_points(group=ENTRY_POINT_GROUP))
        raise ValueError(
            f'no retargeter {name!r} registered under {ENTRY_POINT_GROUP!r}; '
            f'available: {available}')
    return matches[0].load()(config, robot_info)
