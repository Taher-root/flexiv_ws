"""Session setup shared by the teleop nodes.

These are plain functions rather than a node so the mode manager (which takes
over servo arbitration) can reuse them unchanged instead of reimplementing
them. ``servo_session_node`` is only a thin wrapper.
"""

from __future__ import annotations

import time

import rclpy
from moveit_msgs.srv import ServoCommandType
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SessionError(RuntimeError):
    pass


def _spin_until(node, predicate, timeout_sec: float) -> bool:
    end = time.time() + timeout_sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def goto_named_state(node, robot_info, group: str, state_name: str,
                     controller: str, joint_states_topic: str,
                     duration_sec: float = 3.0, timeout_sec: float = 20.0,
                     tolerance: float = 0.05) -> dict:
    """Move ``group`` to an SRDF named state and wait until it arrives.

    Uses the controller's TOPIC interface rather than its action, matching how
    servo commands it, so this exercises the same path teleop will use.

    Waits for a live connection before publishing: a JointTrajectory sent
    before DDS discovery completes is dropped silently, and the arm simply
    never moves.
    """
    targets = robot_info.named_state(group, state_name)
    joints = [j for j in robot_info.group_joints(group) if j in targets]
    if not joints:
        raise SessionError(
            f'named state {state_name!r} shares no joints with group {group!r}')

    state = {}
    sub = node.create_subscription(
        JointState, joint_states_topic,
        lambda m: state.update(zip(m.name, m.position)), 10)
    pub = node.create_publisher(JointTrajectory, f'{controller}/joint_trajectory', 10)

    if not _spin_until(node, lambda: state and pub.get_subscription_count() > 0,
                       timeout_sec):
        node.destroy_subscription(sub)
        raise SessionError(
            f'no joint states on {joint_states_topic!r} or no subscriber on '
            f'{controller}/joint_trajectory; is the controller running?')

    traj = JointTrajectory()
    traj.joint_names = joints
    point = JointTrajectoryPoint()
    point.positions = [targets[j] for j in joints]
    point.time_from_start.sec = int(duration_sec)
    point.time_from_start.nanosec = int((duration_sec % 1.0) * 1e9)
    traj.points = [point]
    pub.publish(traj)

    def arrived():
        return all(abs(state.get(j, 1e9) - targets[j]) < tolerance for j in joints)

    reached = _spin_until(node, arrived, timeout_sec)
    result = {j: state.get(j) for j in joints}
    node.destroy_subscription(sub)
    if not reached:
        raise SessionError(
            f'did not reach {state_name!r} within {timeout_sec}s; at {result}')
    return result


def check_servo_model_matches(node, servo_node: str, group: str,
                              expected_joints, reference=None,
                              timeout_sec: float = 5.0) -> None:
    """Verify servo is modelling the same robot move_group is.

    If servo is launched with different xacro args than the robot bringup, it
    builds a different robot. The first planning-scene update then kills it
    with e.g. "Variable 'gripper' is not known to model", and the only symptom
    anyone sees afterwards is this node reporting a missing service. Checking
    here turns that cascade into one accurate sentence.
    """
    from .robot_info import RobotInfo, RobotInfoError, fetch_descriptions
    try:
        servo_urdf, servo_srdf = fetch_descriptions(
            node, servo_node, timeout_sec=timeout_sec)
    except Exception:  # noqa: BLE001 - servo may already be dead
        return
    try:
        servo = RobotInfo.from_xml(servo_urdf, servo_srdf)
        servo_joints = servo.group_joint_names(group)
    except Exception:  # noqa: BLE001
        return

    if set(servo_joints) != set(expected_joints):
        raise SessionError(
            f'{servo_node} is modelling a DIFFERENT robot than move_group.\n'
            f'  move_group group {group!r}: {sorted(expected_joints)}\n'
            f'  servo      group {group!r}: {sorted(servo_joints)}\n'
            'Servo was built with different xacro arguments than the robot '
            'bringup. Pass the same arm_type/effector_type to both.')

    # Same joints does not mean the same robot. A differing tcp_offset moves
    # the control point while leaving every joint name identical, so the arm
    # goes somewhere other than commanded with nothing logged anywhere.
    if reference is not None:
        try:
            tip = reference.group_tip_link(group)
            joint = reference.parent_joint_of(tip)
            mine = reference.joint_origin(joint)
            theirs = servo.joint_origin(servo.parent_joint_of(tip))
        except RobotInfoError as exc:
            # Say so rather than skipping in silence: a guard that quietly
            # does nothing is worse than no guard, because it looks like a
            # clean bill of health.
            node.get_logger().warn(
                f'could not compare {group!r} tip placement with '
                f'{servo_node}: {exc}')
            return
        if not _close(mine, theirs):
            raise SessionError(
                f'{servo_node} places {tip!r} differently from move_group.\n'
                f'  move_group: xyz={mine[0]} rpy={mine[1]}\n'
                f'  servo     : xyz={theirs[0]} rpy={theirs[1]}\n'
                'The two were built with different tcp_offset values. Pass the '
                'same tcp_offset to the robot bringup and to teleop.')


def _close(a, b, tol: float = 1e-6) -> bool:
    """Compare two ``(xyz, rpy)`` origins componentwise."""
    return all(abs(x - y) < tol
               for side_a, side_b in zip(a, b)
               for x, y in zip(side_a, side_b))


def switch_servo_command_type(node, servo_node: str = '/servo_node',
                              command_type: int = ServoCommandType.Request.POSE,
                              timeout_sec: float = 15.0) -> None:
    """Put servo into a command mode.

    Not optional. Servo silently ignores pose commands until this is called,
    which presents as a servo node that is up, publishing status, reporting no
    warnings, and doing nothing at all.
    """
    client = node.create_client(ServoCommandType, f'{servo_node}/switch_command_type')
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise SessionError(
            f'no switch_command_type service on {servo_node}.\n'
            'The usual cause is that servo crashed at startup because it was '
            'built with different xacro arguments than the robot bringup -- '
            'look further up the log for "is not known to model". Pass the '
            'same arm_type/effector_type to both launches.')
    request = ServoCommandType.Request()
    request.command_type = command_type
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if future.result() is None or not future.result().success:
        raise SessionError(f'{servo_node} refused command_type={command_type}')
