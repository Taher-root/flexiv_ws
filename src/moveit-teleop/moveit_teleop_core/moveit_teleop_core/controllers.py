"""Discover which ros2_control controller drives which joints.

The plan's original config schema asked the user to name the controller that
actuates each group. It turns out not to be necessary: move_group already
carries the whole mapping as parameters, because MoveIt needs it to execute
trajectories at all.

    moveit_simple_controller_manager.controller_names -> [arm_controller, ...]
    moveit_simple_controller_manager.<name>.joints    -> [joint1, ...]

So the controller is derived, not configured, and one more robot-specific key
leaves the user's YAML.
"""

from __future__ import annotations

import rclpy
from rcl_interfaces.srv import GetParameters

PREFIX = 'moveit_simple_controller_manager'


class ControllerLookupError(RuntimeError):
    pass


def _get_params(node, source_node: str, names: list, timeout_sec: float):
    client = node.create_client(GetParameters, f'{source_node}/get_parameters')
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise ControllerLookupError(f'no parameter service on {source_node}')
    request = GetParameters.Request()
    request.names = names
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if future.result() is None:
        raise ControllerLookupError(f'parameter request to {source_node} timed out')
    return future.result().values


def fetch_controller_map(node, source_node: str = '/move_group',
                         timeout_sec: float = 10.0) -> dict:
    """Return ``{controller_name: [joint, ...]}`` as MoveIt understands it."""
    values = _get_params(node, source_node, [f'{PREFIX}.controller_names'], timeout_sec)
    names = list(values[0].string_array_value)
    if not names:
        raise ControllerLookupError(
            f'{source_node} lists no controllers under {PREFIX}.controller_names')

    joint_values = _get_params(
        node, source_node, [f'{PREFIX}.{n}.joints' for n in names], timeout_sec)
    return {name: list(value.string_array_value)
            for name, value in zip(names, joint_values)}


def fetch_joint_limits(node, joints, source_node: str = '/move_group',
                       timeout_sec: float = 10.0) -> dict:
    """Velocity and acceleration limits as MOVEIT understands them.

    MoveIt's ``joint_limits.yaml`` is the authority for how fast a joint may be
    driven, and it can differ sharply from the URDF: on the Piper the URDF says
    5.0 rad/s while MoveIt applies a ``default_velocity_scaling_factor`` of 0.1
    on top, i.e. 0.5 rad/s. Streaming commands built from the URDF number alone
    would move the real arm ten times faster than every planned motion.

    Returns ``{joint: {'max_velocity': v, 'max_acceleration': a}}`` with missing
    entries simply absent, so the caller can fall back per joint.
    """
    prefix = 'robot_description_planning.joint_limits'
    names = []
    for joint in joints:
        names += [f'{prefix}.{joint}.max_velocity',
                  f'{prefix}.{joint}.max_acceleration',
                  f'{prefix}.{joint}.has_velocity_limits',
                  f'{prefix}.{joint}.has_acceleration_limits']
    try:
        values = _get_params(node, source_node, names, timeout_sec)
    except ControllerLookupError:
        return {}

    limits = {}
    for i, joint in enumerate(joints):
        v, a, has_v, has_a = values[4 * i:4 * i + 4]
        entry = {}
        # PARAMETER_NOT_SET is type 0; anything else means the value is real.
        if has_v.type != 0 and has_v.bool_value and v.type != 0:
            entry['max_velocity'] = float(v.double_value)
        if has_a.type != 0 and has_a.bool_value and a.type != 0:
            entry['max_acceleration'] = float(a.double_value)
        if entry:
            limits[joint] = entry
    return limits


def fetch_velocity_scaling(node, source_node: str = '/move_group',
                           timeout_sec: float = 5.0) -> float:
    """MoveIt's ``default_velocity_scaling_factor``, or 1.0 if unset."""
    try:
        values = _get_params(
            node, source_node,
            ['robot_description_planning.default_velocity_scaling_factor'],
            timeout_sec)
    except ControllerLookupError:
        return 1.0
    value = values[0]
    return float(value.double_value) if value.type != 0 else 1.0


def controller_for_joints(controller_map: dict, joints) -> str:
    """Return the controller that covers every joint in ``joints``.

    Requires a superset match rather than an exact one, so a controller that
    also drives extra joints still qualifies. Refuses to choose when several
    match: commanding the wrong controller moves the wrong part of the robot,
    which is not a failure worth guessing through.
    """
    wanted = set(joints)
    matches = [name for name, owned in controller_map.items()
               if wanted and wanted.issubset(set(owned))]
    if not matches:
        raise ControllerLookupError(
            f'no controller covers {sorted(wanted)}; available: '
            + ', '.join(f'{n}={v}' for n, v in controller_map.items()))
    if len(matches) > 1:
        # Prefer the tightest fit; only give up if there is a genuine tie.
        matches.sort(key=lambda n: len(controller_map[n]))
        if len(controller_map[matches[0]]) == len(controller_map[matches[1]]):
            raise ControllerLookupError(
                f'{sorted(wanted)} is covered by several controllers {matches}; '
                'name one explicitly')
    return matches[0]
