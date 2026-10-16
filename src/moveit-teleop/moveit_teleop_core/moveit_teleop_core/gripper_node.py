"""Map a normalized gripper command onto whatever gripper this robot has.

The device reports a single scalar in [0, 1]. Everything needed to turn that
into joint values comes from the robot model:

* which joints the gripper group actuates -> SRDF group (minus mimic joints)
* what open and closed mean               -> SRDF group_states
* which controller drives them            -> move_group's controller parameters

So the same node drives a parallel jaw, a suction cup, or a multi-finger hand
whose SRDF declares open/closed states, with no per-robot code.

Motion is gated on the mode manager: the gripper is as capable of crushing
something during a fault as the arm is of swinging into it, so it obeys the
same watchdog.
"""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_teleop_msgs.msg import TeleopInput, TeleopStatus

from .controllers import controller_for_joints, fetch_controller_map
from .robot_info import RobotInfo, RobotInfoError, fetch_descriptions

INPUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST, depth=1,
    durability=DurabilityPolicy.VOLATILE)


def _resolve_state(robot: RobotInfo, group: str, configured: str,
                   keyword: str) -> dict:
    """Find a named state, falling back to a keyword search.

    The MoveIt Setup Assistant names these inconsistently across robots
    ('open', 'gripper_open', 'hand_open'), so rather than force every user to
    configure it, look for the obvious thing and say clearly what was found.
    """
    available = robot.named_states(group)
    if configured:
        if configured not in available:
            raise RobotInfoError(
                f'group {group!r} has no state {configured!r}; available: {available}')
        return robot.named_state(group, configured)

    matches = [s for s in available if keyword in s.lower()]
    if not matches:
        raise RobotInfoError(
            f'no {keyword!r} state for group {group!r} (available: {available}); '
            f'name one explicitly')
    return robot.named_state(group, sorted(matches, key=len)[0])


class GripperNode(Node):

    def __init__(self):
        super().__init__('gripper_node')

        self.declare_parameter('gripper_group', 'gripper')
        self.declare_parameter('open_state', '')     # '' -> search for *open*
        self.declare_parameter('closed_state', '')   # '' -> search for *close*
        self.declare_parameter('controller', '')     # '' -> derive
        self.declare_parameter('description_source', '/move_group')
        self.declare_parameter('input_topic', '/fake_device/input')
        self.declare_parameter('status_topic', '/mode_manager/status')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('command_time', 0.15)
        self.declare_parameter('deadband', 0.01)
        self.declare_parameter('require_teleop_mode', True)
        self.declare_parameter('joint_states_topic', '/control/joint_states')

        group = self.get_parameter('gripper_group').value
        source = self.get_parameter('description_source').value

        urdf, srdf = fetch_descriptions(self, source)
        robot = RobotInfo.from_xml(urdf, srdf)

        self.joints = robot.actuated_group_joints(group)
        if not self.joints:
            raise RuntimeError(f'gripper group {group!r} actuates no joints')

        open_state = _resolve_state(
            robot, group, self.get_parameter('open_state').value, 'open')
        closed_state = _resolve_state(
            robot, group, self.get_parameter('closed_state').value, 'close')
        self.open_values = [open_state[j] for j in self.joints]
        self.closed_values = [closed_state[j] for j in self.joints]

        controller = self.get_parameter('controller').value
        if not controller:
            controller = controller_for_joints(
                fetch_controller_map(self, source), self.joints)
        self.controller = controller

        self.get_logger().info(
            f"group='{group}' joints={self.joints} controller='{controller}' "
            f'open={self.open_values} closed={self.closed_values} '
            f'(all read from the robot model)')

        self.require_mode = bool(self.get_parameter('require_teleop_mode').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.command_time = float(self.get_parameter('command_time').value)

        self._command = None
        self._last_sent = None
        self._mode = None
        self._measured = {}

        self.pub = self.create_publisher(
            JointTrajectory, f'{controller}/joint_trajectory', 10)
        self.create_subscription(
            TeleopInput, self.get_parameter('input_topic').value,
            self._on_input, INPUT_QOS)
        self.create_subscription(
            TeleopStatus, self.get_parameter('status_topic').value,
            self._on_status, 10)
        self.create_subscription(
            JointState, self.get_parameter('joint_states_topic').value,
            self._on_joint_states, 10)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._tick)

    def _on_input(self, msg: TeleopInput) -> None:
        if msg.gripper_valid:
            self._command = max(0.0, min(1.0, float(msg.gripper)))

    def _on_joint_states(self, msg: JointState) -> None:
        self._measured.update(zip(msg.name, msg.position))

    def _on_status(self, msg: TeleopStatus) -> None:
        was_live = self._mode == TeleopStatus.TELEOP
        self._mode = msg.mode
        if was_live and msg.mode != TeleopStatus.TELEOP:
            self._hold()

    def _hold(self) -> None:
        """Truncate any in-flight motion by commanding the measured position.

        Simply ceasing to publish is not a stop: the controller keeps executing
        the last trajectory it was given, so the gripper carries on closing for
        the remainder of that command. Re-commanding where the fingers actually
        ARE right now overrides it and is the gripper's equivalent of servo's
        smooth halt.
        """
        if not all(j in self._measured for j in self.joints):
            return
        traj = JointTrajectory()
        traj.joint_names = list(self.joints)
        point = JointTrajectoryPoint()
        point.positions = [float(self._measured[j]) for j in self.joints]
        point.time_from_start.nanosec = 10_000_000    # 10 ms: stop, do not glide
        traj.points = [point]
        self.pub.publish(traj)
        self._last_sent = None    # force a fresh command on re-engage
        self.get_logger().info(f'held gripper at {point.positions}')

    def _allowed(self) -> bool:
        if not self.require_mode:
            return True
        # No status at all means the mode manager is absent or dead. Refusing
        # to actuate is the safe reading: an ungoverned gripper has no watchdog.
        return self._mode == TeleopStatus.TELEOP

    def _tick(self) -> None:
        if self._command is None or not self._allowed():
            return
        # Rate-limit on change: the controller does not need a fresh trajectory
        # 20 times a second to hold a position it already reached.
        if self._last_sent is not None and \
                abs(self._command - self._last_sent) < self.deadband:
            return
        self._last_sent = self._command

        traj = JointTrajectory()
        traj.joint_names = list(self.joints)
        point = JointTrajectoryPoint()
        point.positions = [
            c + (o - c) * self._command
            for o, c in zip(self.open_values, self.closed_values)
        ]
        point.time_from_start.sec = int(self.command_time)
        point.time_from_start.nanosec = int((self.command_time % 1.0) * 1e9)
        traj.points = [point]
        self.pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GripperNode()
    except Exception as exc:  # noqa: BLE001
        rclpy.logging.get_logger('gripper_node').fatal(str(exc))
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
