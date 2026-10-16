"""Everything the export needs to know about a robot, from files that already exist.

The exporter reads the SAME robot config the teleop launch does --
``<arm>_moveit_config/config/teleop.yaml`` -- and derives the dataset's shape
from the MoveIt model it names. Nothing about a particular arm is written into
this package.

Derived from the robot model (never configure these)
----------------------------------------------------
=====================  ===================================================
arm joints, in order   SRDF group ``move_group``, walked base -> tip
gripper joints         SRDF group ``gripper.group``, minus mimic joints
units                  URDF joint type: revolute/continuous -> rad,
                       prismatic -> m
robot name             URDF ``<robot name>``
=====================  ===================================================

This is not cosmetic. The previous exporter decided which joints were the
gripper by looking for "gripper" in the joint name -- right on the Piper, and
wrong on a Franka, whose finger joint is ``fr3_finger_joint1``.

Required from the user (cannot be derived, so the export stops without them)
----------------------------------------------------------------------------
``move_group``          which SRDF group is the arm
``files.preview_urdf``  the robot description, as xacro or URDF
``files.preview_srdf``  the semantic description, as xacro or SRDF
``files.preview_xacro_args``  the xacro arguments the robot bringup used

The files are resolved relative to teleop.yaml, exactly as the teleop launch
resolves them. They are needed because an export runs OFFLINE: there is no
running move_group to ask, and a recorded bag does not contain the model.

Optional choices (the ``dataset:`` section; defaults below)
-----------------------------------------------------------
See :data:`DATASET_DEFAULTS`.
"""

from __future__ import annotations

import copy
import os
import shlex
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

import yaml

#: Where a robot keeps its teleop config inside its MoveIt config package.
PACKAGE_CONFIG = Path('config') / 'teleop.yaml'

#: The ``dataset:`` section of teleop.yaml. Every key is optional; an explicit
#: `convert` flag beats the config, which beats these.
DATASET_DEFAULTS = {
    # LeRobot's info.json robot_type. None -> the URDF robot name.
    'robot_type': None,
    # Recording stream names (from recording.yaml) that become state and action.
    'state_stream': 'joint_states',
    'action_stream': 'joint_commands',
    # 'teleop' keeps first engage -> last disengage; 'none' keeps everything.
    'trim': 'teleop',
    # Camera frames: 'WxH' or 'native'. Both GR00T and pi0.5 resize to 224.
    'image_size': '640x360',
    # h264 plays in any player; many desktop players cannot play AV1 (VLC 3.0
    # would not play these files). Both GR00T and openpi decode either.
    'vcodec': 'h264',
    'gr00t': {
        # Steps of action predicted per sample. At the camera's fps.
        'action_horizon': 16,
        # How each group's action is represented: relative (delta from the
        # current state) or absolute. GR00T's SO-100 reference config uses
        # relative for the arm and absolute for the gripper.
        'arm_action': 'relative',
        'gripper_action': 'absolute',
    },
}

CHOICES = {
    'trim': ('teleop', 'none'),
    'vcodec': ('libsvtav1', 'h264'),
    'gr00t.arm_action': ('relative', 'absolute'),
    'gr00t.gripper_action': ('relative', 'absolute'),
}

UNITS = {'revolute': 'rad', 'continuous': 'rad', 'prismatic': 'm'}


class RobotConfigError(ValueError):
    """The robot config cannot describe a dataset; the message says what to add."""


@dataclass
class JointGroup:
    """One named slice of the state and action vectors."""

    name: str            # 'arm' or 'gripper' -- the key GR00T's modality.json uses
    srdf_group: str
    joints: list
    units: list


@dataclass
class RobotModel:
    name: str
    groups: list
    config_path: Path
    dataset: dict = field(default_factory=dict)

    @property
    def joint_names(self) -> list:
        return [j for g in self.groups for j in g.joints]

    @property
    def robot_type(self) -> str:
        return self.dataset.get('robot_type') or self.name

    def describe(self) -> dict:
        start, spans = 0, []
        for g in self.groups:
            spans.append({'name': g.name, 'srdf_group': g.srdf_group,
                          'start': start, 'end': start + len(g.joints),
                          'joints': list(g.joints), 'units': list(g.units)})
            start += len(g.joints)
        return {'robot_name': self.name, 'robot_type': self.robot_type,
                'config': str(self.config_path), 'groups': spans}


# ---- the dataset section --------------------------------------------------

def _leaves(mapping, prefix=''):
    for key, value in (mapping or {}).items():
        path = f'{prefix}.{key}' if prefix else str(key)
        if isinstance(value, dict):
            yield from _leaves(value, path)
        else:
            yield path, value


def dataset_settings(raw) -> dict:
    """Merge a ``dataset:`` section over the defaults, refusing unknown keys.

    A misspelled key must fail loudly: the teleop launch ignores this section
    entirely, so nothing else would ever notice.
    """
    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        raise RobotConfigError('dataset: must be a mapping')
    known = dict(_leaves(DATASET_DEFAULTS))
    unknown = sorted(k for k, _ in _leaves(raw) if k not in known)
    if unknown:
        raise RobotConfigError(
            f'dataset: sets {unknown}, which the exporter does not read. '
            f'Known: {sorted(known)}')
    merged = copy.deepcopy(DATASET_DEFAULTS)
    for key, value in _leaves(raw):
        node = merged
        *parents, leaf = key.split('.')
        for part in parents:
            node = node[part]
        node[leaf] = value
    for key, allowed in CHOICES.items():
        node = merged
        for part in key.split('.'):
            node = node[part]
        if node not in allowed:
            raise RobotConfigError(f'dataset.{key} is {node!r}; expected one of {list(allowed)}')
    horizon = merged['gr00t']['action_horizon']
    if not isinstance(horizon, int) or horizon < 1:
        raise RobotConfigError(f'dataset.gr00t.action_horizon must be a positive integer, got {horizon!r}')
    return merged


# ---- locating and reading -------------------------------------------------

def config_path(package: str = None, path=None) -> Path:
    """teleop.yaml from an explicit path, or from a package's share directory."""
    if path:
        p = Path(path).expanduser()
        if not p.is_file():
            raise RobotConfigError(f'robot config not found: {p}')
        return p
    if not package:
        raise RobotConfigError(
            'name the robot: --robot-config-package <arm>_moveit_config (reads '
            f'<share>/{PACKAGE_CONFIG}), or --robot-config /path/to/teleop.yaml')
    from ament_index_python.packages import (
        PackageNotFoundError, get_package_share_directory,
    )
    try:
        share = Path(get_package_share_directory(package))
    except PackageNotFoundError as exc:
        raise RobotConfigError(
            f'{package!r} is not on the ament prefix path; source the workspace '
            'that builds it') from exc
    p = share / PACKAGE_CONFIG
    if not p.is_file():
        raise RobotConfigError(
            f'{package!r} has no {PACKAGE_CONFIG}. Start one from '
            'moveit_teleop_bringup/config/samples/template_teleop.yaml.')
    return p


def _require(root: dict, dotted: str, why: str, cfg: Path):
    node = root
    for part in dotted.split('.'):
        if not isinstance(node, dict) or part not in node or node[part] in (None, ''):
            raise RobotConfigError(
                f'{cfg}: `{dotted}` is REQUIRED to export datasets -- {why}')
        node = node[part]
    return node


def render(description: Path, xacro_args: str) -> str:
    """A URDF or SRDF as XML text, running xacro only when the file is xacro."""
    if not description.is_file():
        raise RobotConfigError(f'robot description not found: {description}')
    if description.suffix != '.xacro':
        return description.read_text()
    try:
        return subprocess.run(['xacro', str(description), *shlex.split(xacro_args or '')],
                              check=True, capture_output=True, text=True).stdout
    except FileNotFoundError as exc:
        raise RobotConfigError('xacro is not installed; source ROS first') from exc
    except subprocess.CalledProcessError as exc:
        raise RobotConfigError(
            f'xacro failed on {description} with args {xacro_args!r}:\n'
            f'{exc.stderr.strip()[-800:]}') from exc


def load_robot_model(package: str = None, path=None) -> RobotModel:
    """Read a robot's teleop.yaml and derive the dataset's joint layout."""
    from moveit_teleop_core.robot_info import RobotInfo, RobotInfoError

    cfg = config_path(package, path)
    document = yaml.safe_load(cfg.read_text()) or {}
    root = document.get('moveit_teleop')
    if not isinstance(root, dict):
        raise RobotConfigError(f"{cfg} has no top-level 'moveit_teleop' mapping")

    arm_group = _require(root, 'move_group', 'the SRDF group that is the arm', cfg)
    why_files = ('an export runs offline, so the robot model must come from files '
                 '(relative to teleop.yaml, as the teleop launch reads them)')
    here = cfg.parent

    def resolve(key):
        value = _require(root, key, why_files, cfg)
        p = Path(os.path.expanduser(value))
        return p if p.is_absolute() else (here / p).resolve()

    urdf_file, srdf_file = resolve('files.preview_urdf'), resolve('files.preview_srdf')
    args = (root.get('files') or {}).get('preview_xacro_args') or ''
    robot = RobotInfo.from_xml(render(urdf_file, args), render(srdf_file, args))

    types = {j.get('name'): j.get('type') for j in robot.urdf.findall('joint')}

    def group(name, srdf_group, joints):
        if not joints:
            raise RobotConfigError(f"SRDF group {srdf_group!r} actuates no joints")
        units = []
        for j in joints:
            if types.get(j) not in UNITS:
                raise RobotConfigError(
                    f'joint {j!r} in {srdf_group!r} is {types.get(j)!r}; only '
                    f'{sorted(UNITS)} joints can be exported as positions')
            units.append(UNITS[types[j]])
        return JointGroup(name, srdf_group, list(joints), units)

    try:
        groups = [group('arm', arm_group, robot.group_joints(arm_group))]
        gripper = root.get('gripper') or {}
        if gripper.get('enabled'):
            grip_group = _require(root, 'gripper.group',
                                  'gripper.enabled is true, so name its SRDF group', cfg)
            grip = robot.actuated_group_joints(grip_group)
            overlap = set(grip) & set(groups[0].joints)
            if overlap:
                raise RobotConfigError(f'{sorted(overlap)} are in both the arm and gripper groups')
            groups.append(group('gripper', grip_group, grip))
    except RobotInfoError as exc:
        raise RobotConfigError(f'{cfg}: {exc}') from exc

    return RobotModel(name=robot.urdf.get('name') or 'robot', groups=groups,
                      config_path=cfg, dataset=dataset_settings(root.get('dataset')))
