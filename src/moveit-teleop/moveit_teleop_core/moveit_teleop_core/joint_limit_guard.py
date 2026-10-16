"""Clamp streamed joint commands to the robot's own position limits.

Nothing else in the servo path enforces position limits:

* ``mock_components/GenericSystem`` echoes whatever it is given
* ``JointTrajectoryController`` does not clamp positions
* ``moveit_servo`` halts a joint that has REACHED its bound, but if a command
  ever places one beyond it, halting is precisely the wrong response -- zero
  velocity means the joint can never come back, and the arm jams there

That last case is not hypothetical: a Piper wrist reached 47 degrees outside
its limit during teleoperation and then refused to move at all, with servo
logging "Joint position limit reached" forever.

This node sits between servo and the controller and simply refuses to pass a
command outside the model's limits. Limits are read from robot_description, so
it works on any robot without configuration.
"""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

from .robot_info import RobotInfo, fetch_descriptions


class JointLimitGuard(Node):

    def __init__(self):
        super().__init__('joint_limit_guard')

        self.declare_parameter('description_source', '/move_group')
        self.declare_parameter('input_topic', 'servo/joint_trajectory_raw')
        self.declare_parameter('output_topic', 'arm_controller/joint_trajectory')
        # Keep a hair inside the bound. Landing exactly ON it still trips
        # servo's "limit reached" halt, which is the state we are avoiding.
        self.declare_parameter('margin', 0.01)
        self.declare_parameter('report_period', 2.0)

        source = self.get_parameter('description_source').value
        urdf, srdf = fetch_descriptions(self, source)
        robot = RobotInfo.from_xml(urdf, srdf)

        self.margin = float(self.get_parameter('margin').value)
        self.limits = {}
        for joint in robot.urdf.findall('joint'):
            name = joint.get('name')
            if joint.get('type') not in ('revolute', 'prismatic'):
                continue
            lower, upper = robot.joint_limits(name)
            if lower is None or upper is None:
                continue
            # A margin wider than the joint's travel would invert the bounds.
            margin = min(self.margin, max(0.0, (upper - lower) / 2.0 - 1e-6))
            self.limits[name] = (lower + margin, upper - margin)

        self._clamped_total = 0
        self._offenders = {}
        self._last_report = 0.0
        self.report_period = float(self.get_parameter('report_period').value)

        self.pub = self.create_publisher(
            JointTrajectory, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            JointTrajectory, self.get_parameter('input_topic').value,
            self._on_traj, 10)

        self.get_logger().info(
            f"guarding {len(self.limits)} joints: "
            f"'{self.get_parameter('input_topic').value}' -> "
            f"'{self.get_parameter('output_topic').value}' "
            f'(limits from the robot model, margin {self.margin})')

    def _on_traj(self, msg: JointTrajectory) -> None:
        clamped_here = False
        for point in msg.points:
            positions = list(point.positions)
            for i, name in enumerate(msg.joint_names):
                if i >= len(positions) or name not in self.limits:
                    continue
                lower, upper = self.limits[name]
                value = positions[i]
                if value < lower or value > upper:
                    positions[i] = min(max(value, lower), upper)
                    self._offenders[name] = self._offenders.get(name, 0) + 1
                    clamped_here = True
            point.positions = positions
        if clamped_here:
            self._clamped_total += 1
        self.pub.publish(msg)
        self._maybe_report()

    def _maybe_report(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self._offenders or now - self._last_report < self.report_period:
            return
        self._last_report = now
        worst = ', '.join(f'{n}x{c}' for n, c in
                          sorted(self._offenders.items(), key=lambda kv: -kv[1]))
        self.get_logger().warn(
            f'clamped {self._clamped_total} commands to joint limits ({worst}). '
            'The arm is being driven at its limits; reduce translation_scale or '
            'reposition, but it can no longer jam outside its range.')
        self._offenders = {}


def main(args=None):
    rclpy.init(args=args)
    try:
        node = JointLimitGuard()
    except Exception as exc:  # noqa: BLE001
        rclpy.logging.get_logger('joint_limit_guard').fatal(str(exc))
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
