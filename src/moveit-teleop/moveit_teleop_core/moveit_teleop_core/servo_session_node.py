"""Prepare the robot for servo teleoperation, then hold.

Two jobs, both of which must happen before a pose target will do anything:

1. Move to the SRDF ``ready_state``. The Piper's zero pose sits on two joint
   limits and is a fully-extended elbow singularity; servo answers every
   command from there with an emergency stop while still publishing at full
   rate, so the arm looks live and does nothing.
2. Call ``switch_command_type(POSE)``. Servo ignores pose commands until this
   is done, with no error anywhere.

The mode manager absorbs both in step 5; this node exists so the loop can be
closed and soak-tested first. It uses the same helpers the mode manager will.
"""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .controllers import controller_for_joints, fetch_controller_map
from .robot_info import RobotInfo, fetch_descriptions
from .session import goto_named_state, switch_servo_command_type


class ServoSessionNode(Node):

    def __init__(self):
        super().__init__('servo_session')

        self.declare_parameter('move_group', 'arm')
        self.declare_parameter('ready_state', 'ready_pose')
        self.declare_parameter('description_source', '/move_group')
        self.declare_parameter('servo_node', '/servo_node')
        self.declare_parameter('joint_states_topic', '/control/joint_states')
        self.declare_parameter('controller', '')   # '' -> derive from move_group
        self.declare_parameter('goto_ready', True)

        group = self.get_parameter('move_group').value
        source = self.get_parameter('description_source').value

        urdf, srdf = fetch_descriptions(self, source)
        robot = RobotInfo.from_xml(urdf, srdf)
        joints = robot.group_joints(group)

        controller = self.get_parameter('controller').value
        if not controller:
            controller = controller_for_joints(
                fetch_controller_map(self, source), joints)
            self.get_logger().info(
                f"controller '{controller}' derived from move_group, not config")

        if self.get_parameter('goto_ready').value:
            state = self.get_parameter('ready_state').value
            available = robot.named_states(group)
            if state not in available:
                raise RuntimeError(
                    f'group {group!r} has no named state {state!r}; '
                    f'available: {available}. Add one to the SRDF -- servoing '
                    f'from a singular posture will not work.')
            self.get_logger().info(f"moving to '{state}' before enabling servo")
            reached = goto_named_state(
                self, robot, group, state, controller,
                self.get_parameter('joint_states_topic').value)
            self.get_logger().info(
                'at ' + str({k: round(v, 3) for k, v in reached.items()}))

        servo = self.get_parameter('servo_node').value
        switch_servo_command_type(self, servo)
        self.get_logger().info(f'{servo} switched to POSE; teleop targets are live')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ServoSessionNode()
    except Exception as exc:  # noqa: BLE001
        rclpy.logging.get_logger('servo_session').fatal(str(exc))
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
