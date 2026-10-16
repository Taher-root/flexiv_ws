"""Make the robot's own config file the source of truth for teleop.

Where the file lives
--------------------
With the robot, not with the teleop stack. The intended home is the arm's
MoveIt config package::

    <arm>_moveit_config/config/teleop.yaml

and it is selected by package name rather than by path::

    ros2 launch moveit_teleop_bringup teleop.launch.py robot_config_package:=agx_arm_moveit

``robot_config:=/some/path.yaml`` still works and wins, for a file that is not
in a package yet. Samples for both shipped arms live in
``moveit_teleop_bringup/config/samples/``.

Companion files -- the planning scene, the RViz view, the xacros the replay
preview robot is built from -- are named INSIDE that file, relative to it, so
one ``robot_config_package`` is the entire robot-specific surface and no arm
needs a wrapper launch file.

The problem the sentinel solves
-------------------------------
A ``DeclareLaunchArgument`` default ALWAYS wins over a config file, because
launch cannot distinguish "the caller did not pass this" from "the caller
passed exactly the default". Any launch file that declares real defaults and
also reads a YAML has a config file that silently does nothing -- which is
precisely what happened to piper.yaml.

So every argument the config file owns is declared with the sentinel ``UNSET``
instead of a real default. Resolution order is then unambiguous:

    explicit launch argument  >  config file  >  FALLBACKS below

``FALLBACKS`` exists only so that a config file omitting a key still launches.
It is not where you tune anything; the config file is.

Some values depend on which solver is running -- full IK returns the closest
FEASIBLE pose where servo does not, so it can be driven harder. Those live
under ``retargeting.per_solver.<solver>`` and override the common value.
"""

import os

import yaml

UNSET = '<config>'

#: Conventional location of the teleop config inside a robot's MoveIt config
#: package, used by ``robot_config_package``.
PACKAGE_CONFIG = os.path.join('config', 'teleop.yaml')

# arg name -> dotted path inside the 'moveit_teleop' mapping.
CONFIG_KEYS = {
    'move_group': 'move_group',
    'ready_state': 'ready_state',
    'nominal_state': 'nominal_state',

    'pattern': 'device.pattern',
    'period': 'device.period',
    'radius': 'device.radius',

    'translation_scale': 'retargeting.translation_scale',
    'rotation_scale': 'retargeting.rotation_scale',
    'translation_ref': 'retargeting.translation_ref',
    'rotation_ref': 'retargeting.rotation_ref',
    'smoothing_alpha': 'retargeting.smoothing_alpha',
    'max_translation_step': 'retargeting.max_translation_step',
    'max_target_lead': 'retargeting.max_target_lead',
    'max_target_lead_angle': 'retargeting.max_target_lead_angle',
    'external_motion_threshold': 'retargeting.external_motion_threshold',
    # Was missing, and fr3_teleop.yaml set it -- so it was read by nobody and
    # survived only because the launch default happened to be the same number.
    'max_command_step': 'retargeting.max_command_step',

    'solve_rate': 'ik.solve_rate',
    'weight_position': 'ik.weight_position',
    'weight_orientation': 'ik.weight_orientation',
    'weight_regularization': 'ik.weight_regularization',
    'weight_smoothness': 'ik.weight_smoothness',
    'rate_limit_scale': 'ik.rate_limit_scale',
    'max_joint_jump': 'ik.max_joint_jump',
    'jump_retry_smoothness': 'ik.jump_retry_smoothness',
    'collision_policy': 'ik.collision_policy',

    'gripper_group': 'gripper.group',
    'use_gripper': 'gripper.enabled',
    'arm_controller': 'controllers.arm',

    # Not a path: the xacro arguments the preview robot is built with.
    'replay_xacro_args': 'files.preview_xacro_args',

    'input_timeout': 'safety.input_timeout',
    'hardware_gate_enabled': 'safety.hardware_gate.enabled',
    'hardware_gate_service': 'safety.hardware_gate.service',
    # Whether that gate must also be open while move_group executes a planned
    # trajectory. A motion-enable gate: yes. A gate that selects a streaming
    # mode the driver's trajectory action refuses to run alongside: no.
    'hardware_gate_open_for_planning':
        'safety.hardware_gate.open_for_planning',
    # A per-robot capability question, not a preference: does this arm's MoveIt
    # stack execute trajectories? Every MoveIt config that has a controller
    # manager does, which is why the default is true.
    'home_via_moveit': 'safety.home_via_moveit',
}

#: Arguments naming a FILE. Resolved relative to the config file's own
#: directory, so a robot's config package is self-contained and movable.
FILE_KEYS = {
    'scene_config': 'files.scene',
    'rviz_config': 'files.rviz',
    'replay_urdf_xacro': 'files.preview_urdf',
    'replay_srdf_xacro': 'files.preview_srdf',
}

# Keys that retargeting.per_solver.<solver> may override.
PER_SOLVER_SECTION = 'retargeting.per_solver'

#: Sections of teleop.yaml that belong to ANOTHER reader, which validates them
#: itself. The launch neither reads nor rejects them.
#:   dataset -- moveit_teleop_record convert (see robot_model.DATASET_DEFAULTS)
EXTERNAL_SECTIONS = ('dataset',)

# Only so an incomplete config file still launches. Tune in the config file.
FALLBACKS = {
    'move_group': 'arm',
    'ready_state': 'ready_pose',
    'nominal_state': 'ready_pose',
    'pattern': 'circle',
    'period': '6.0',
    'radius': '0.1',
    'translation_scale': '0.5',
    'rotation_scale': '1.0',
    'translation_ref': 'base',
    'rotation_ref': 'base',
    'smoothing_alpha': '1.0',
    'max_translation_step': '0.05',
    'max_target_lead': '0.05',
    'max_target_lead_angle': '0.5',
    'external_motion_threshold': '0.15',
    'max_command_step': '0.03',
    'solve_rate': '90.0',
    'weight_position': '50.0',
    'weight_orientation': '10.0',
    'weight_regularization': '0.005',
    'weight_smoothness': '0.1',
    'rate_limit_scale': '0.3',
    'max_joint_jump': '0.35',
    'jump_retry_smoothness': '50.0',
    'collision_policy': 'stop',
    'gripper_group': 'gripper',
    'use_gripper': 'false',
    'arm_controller': 'arm_controller',
    'replay_xacro_args': '',
    'input_timeout': '0.1',
    'home_via_moveit': 'true',
    'hardware_gate_enabled': 'false',
    'hardware_gate_open_for_planning': 'true',
    'hardware_gate_service': '/control_enable',
}

#: File arguments a config file may omit. '' means "this feature is off":
#: no scene declares no obstacles, no preview xacro disables the replay
#: preview robot. The RViz view falls back to the stack's own generic one.
FILE_FALLBACKS = {
    'scene_config': '',
    'rviz_config': '<bringup>/config/teleop.rviz',
    'replay_urdf_xacro': '',
    'replay_srdf_xacro': '',
}

# Enumerated settings. Checked here rather than by DeclareLaunchArgument's
# `choices`, which cannot hold the sentinel and, more importantly, never saw
# values that came from the config file at all.
CHOICES = {
    'translation_ref': ('base', 'device', 'tool'),
    'rotation_ref': ('base', 'device', 'tool'),
    'collision_policy': ('none', 'stop'),
    'use_gripper': ('true', 'false'),
    'hardware_gate_enabled': ('true', 'false'),
    'home_via_moveit': ('true', 'false'),
    'hardware_gate_open_for_planning': ('true', 'false'),
}


class TeleopConfigError(RuntimeError):
    pass


def _stringify(value):
    """Launch configurations are strings; YAML gives us real types."""
    if value is None:
        return ''
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (list, tuple)):
        return '[' + ', '.join(str(v) for v in value) + ']'
    return str(value)


def _dig(mapping, dotted):
    node = mapping
    for part in dotted.split('.'):
        if not isinstance(node, dict) or part not in node:
            return None, False
        node = node[part]
    return node, True


def _leaves(mapping, prefix=''):
    """Every dotted leaf path in a nested mapping."""
    for key, value in (mapping or {}).items():
        path = f'{prefix}.{key}' if prefix else str(key)
        if isinstance(value, dict) and value:
            yield from _leaves(value, path)
        else:
            yield path


def _check_unknown_keys(root, path):
    """Refuse a config file containing settings nothing reads.

    Without this a typo'd or renamed key is simply skipped, the launch comes up
    looking healthy and the robot runs on a value the file appears to set. That
    is not hypothetical: fr3_teleop.yaml set ``retargeting.max_command_step``
    for weeks while the resolver had no such key.
    """
    known = set(CONFIG_KEYS.values()) | set(FILE_KEYS.values())
    unknown = []
    for leaf in _leaves(root):
        if leaf in known:
            continue
        # per_solver sections are validated separately, against CONFIG_KEYS.
        if leaf.startswith(f'{PER_SOLVER_SECTION}.'):
            continue
        if leaf.split('.')[0] in EXTERNAL_SECTIONS:
            continue
        unknown.append(leaf)
    if unknown:
        raise TeleopConfigError(
            f'{path} sets {sorted(unknown)}, which nothing reads. Either the '
            'key is misspelled or it belongs in another file (obstacles go in '
            'the planning-scene YAML, recorded topics in recording.yaml). '
            f'Known settings: {sorted(known)}')


def load_config(path, solver):
    """Read the config file and flatten it to {arg_name: string}.

    Missing file is an error rather than a silent fall back to defaults: a
    typo'd path that quietly launched a differently-tuned robot is exactly the
    failure this whole change exists to remove.
    """
    path = os.path.expanduser(path)
    if not os.path.isfile(path):
        raise TeleopConfigError(f'teleop config file not found: {path}')

    with open(path, 'r', encoding='utf-8') as handle:
        document = yaml.safe_load(handle) or {}

    root = document.get('moveit_teleop')
    if not isinstance(root, dict):
        raise TeleopConfigError(
            f"{path} has no top-level 'moveit_teleop' mapping")

    _check_unknown_keys(root, path)

    here = os.path.dirname(os.path.abspath(path))
    resolved = {}
    for arg, dotted in CONFIG_KEYS.items():
        value, found = _dig(root, dotted)
        if found:
            resolved[arg] = _stringify(value)
    for arg, dotted in FILE_KEYS.items():
        value, found = _dig(root, dotted)
        if not found:
            continue
        text = _stringify(value)
        # Relative names are relative to the CONFIG FILE, so a robot's config
        # package can be copied or renamed without editing any path in it.
        resolved[arg] = (text if not text or os.path.isabs(text)
                         else os.path.normpath(os.path.join(here, text)))

    # Solver-specific overrides win over the common retargeting values.
    per_solver, found = _dig(root, PER_SOLVER_SECTION)
    if found and isinstance(per_solver, dict):
        section = per_solver.get(solver) or {}
        if not isinstance(section, dict):
            raise TeleopConfigError(
                f'{PER_SOLVER_SECTION}.{solver} must be a mapping')
        for key, value in section.items():
            if key not in CONFIG_KEYS:
                raise TeleopConfigError(
                    f'{PER_SOLVER_SECTION}.{solver}.{key} is not a known '
                    f'setting; expected one of {sorted(CONFIG_KEYS)}')
            resolved[key] = _stringify(value)

    return resolved


def resolve_config_path(configurations):
    """Settle robot_config from robot_config_package, if one was given.

    An explicit ``robot_config`` wins, so a file that is not in a package yet
    can still be pointed at directly.
    """
    explicit = configurations.get('robot_config', UNSET)
    if explicit != UNSET and explicit:
        return explicit

    package = configurations.get('robot_config_package', '')
    if not package:
        raise TeleopConfigError(
            'no robot config: pass robot_config_package:=<arm>_moveit_config '
            '(which reads <share>/' + PACKAGE_CONFIG + ') or '
            'robot_config:=/path/to/teleop.yaml. Samples for the shipped arms '
            'are in moveit_teleop_bringup/config/samples/.')

    from ament_index_python.packages import (
        PackageNotFoundError, get_package_share_directory,
    )
    try:
        share = get_package_share_directory(package)
    except PackageNotFoundError as exc:
        raise TeleopConfigError(
            f'robot_config_package is {package!r}, which is not a package on '
            'the ament prefix path. Did you source the workspace that '
            'contains it?') from exc

    path = os.path.join(share, PACKAGE_CONFIG)
    if not os.path.isfile(path):
        raise TeleopConfigError(
            f'package {package!r} has no {PACKAGE_CONFIG}. Add one (start from '
            'moveit_teleop_bringup/config/samples/) and make sure its install '
            "rule copies the config directory into the package's share.")
    return path


def resolve(context):
    """Fill every UNSET launch configuration from the config file.

    Mutates ``context.launch_configurations`` in place, so every Node and
    IncludeLaunchDescription visited AFTER this action sees resolved values
    and needs no change. Returns nothing to add to the launch description.
    """
    configurations = context.launch_configurations
    path = resolve_config_path(configurations)
    configurations['robot_config'] = path
    solver = configurations['solver']

    from_file = load_config(path, solver)

    # Before the unknown-argument check below, because episodes_dir is the
    # one sentinel this file does NOT own: it comes from recording.yaml, so
    # that the recorder and the replay panel cannot point at different
    # directories.
    _resolve_episodes_dir(configurations)

    owned = set(CONFIG_KEYS) | set(FILE_KEYS)
    unknown = [a for a, v in configurations.items()
               if v == UNSET and a not in owned]
    if unknown:
        raise TeleopConfigError(
            f'launch arguments {sorted(unknown)} are declared UNSET but the '
            'config resolver does not know them; add them to CONFIG_KEYS')

    for arg in CONFIG_KEYS:
        if configurations.get(arg) != UNSET:
            continue  # explicitly passed on the command line -- it wins
        configurations[arg] = from_file.get(arg, FALLBACKS[arg])

    for arg in FILE_KEYS:
        if configurations.get(arg) != UNSET:
            continue
        configurations[arg] = from_file.get(arg, FILE_FALLBACKS[arg])
    _expand_bringup_share(configurations)

    for arg, allowed in CHOICES.items():
        value = configurations.get(arg)
        if value not in allowed:
            raise TeleopConfigError(
                f'{arg} is {value!r}; expected one of {list(allowed)}')

    for arg in FILE_KEYS:
        value = configurations.get(arg)
        if value and not os.path.isfile(os.path.expanduser(value)):
            raise TeleopConfigError(
                f'{arg} is {value!r}, which does not exist. It was named by '
                f'{FILE_KEYS[arg]!r} in {path} and resolved relative to that '
                'file.')

    configurations['topic_substitutions'] = _topic_substitutions(configurations)
    return []


def _expand_bringup_share(configurations) -> None:
    """Turn the '<bringup>' marker in FILE_FALLBACKS into a real path."""
    marker = '<bringup>'
    share = None
    for arg in FILE_KEYS:
        value = configurations.get(arg, '')
        if not value or marker not in value:
            continue
        if share is None:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('moveit_teleop_bringup')
        configurations[arg] = value.replace(marker, share)


def _resolve_episodes_dir(configurations) -> None:
    """Point the replay panel at wherever the RECORDER actually writes.

    recording.yaml owns output_dir, and the replay player had its own
    episodes_dir argument. Two settings for one directory: change the
    recorder's and the panel silently keeps listing the old location, so
    every episode you record is invisible and the stale ones still show.
    Nothing errors, because both halves are individually fine.

    An explicit episodes_dir still wins, for reviewing an archive.
    """
    if configurations.get('episodes_dir') != UNSET:
        return
    path = configurations.get('recording_config', '')
    output = ''
    if path and os.path.isfile(path):
        with open(path, 'r', encoding='utf-8') as handle:
            raw = yaml.safe_load(handle) or {}
        output = (raw.get('recording') or {}).get('output_dir', '') or ''
    configurations['episodes_dir'] = output or '~/teleop_episodes'


def _topic_substitutions(configurations):
    """Topics that depend on the SESSION rather than on the robot.

    Which topic carries the action, and which carries the solver's status,
    follow from the solver in use; the device input topic follows from the
    device. recording.yaml refers to all three as ``${name}`` so that it can
    describe what a training sample CONTAINS without also having to know how
    this particular launch is wired.

    Returned as the text of a Python list literal, because that is what
    ParameterValue(value_type=List[str]) can actually coerce -- handing launch
    a list of substitutions gets them concatenated into a single string.
    """
    ik = configurations['solver'] == 'ik'
    pairs = {
        'target_topic': ('/ik_node/pose_target_cmds' if ik
                         else '/servo_node/pose_target_cmds'),
        'input_topic': configurations['input_topic'],
        'solver_status_topic': '/ik_node/status' if ik else '/servo_node/status',
        # What this launch treats as the robot's MEASURED state -- the
        # driver's feedback on hardware, the command echo on mock hardware.
        # The recorder's observation stream follows it, so a hardware episode
        # records encoder readings rather than the command bus.
        'joint_states_topic': configurations['joint_states_topic'],
    }
    return repr([f'{name}={topic}' for name, topic in pairs.items()])
