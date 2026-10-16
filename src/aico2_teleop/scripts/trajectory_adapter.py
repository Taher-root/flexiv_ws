#!/usr/bin/env python3
"""Adapt the moveit-teleop JTC/gate interface to an AICO2 Flexiv NRT driver.

Parameterized for left or right arm via joint_names. Short IK commands stream
unchanged; longer home trajectories are interpolated from measured feedback
(the driver's servo input ignores time_from_start).

Run in the arm's namespace (left_arm or right_arm) so all topics resolve
relative to that namespace:

    /<ns>/joint_trajectory  ->  /<ns>/servo_joint_command
    /<ns>/control_enable    ->  /<ns>/set_teleop_mode
    TeleopInput.gripper     ->  /<ns>/gripper_command
"""
import math
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from moveit_teleop_msgs.msg import TeleopInput
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def duration(point):
    return point.time_from_start.sec + point.time_from_start.nanosec * 1e-9


def sample(start, end, elapsed, seconds):
    """Cubic ease-in/out, with zero velocity at both ends."""
    u = min(1.0, max(0.0, elapsed / seconds))
    blend = u * u * (3.0 - 2.0 * u)
    return ([a + blend * (b - a) for a, b in zip(start, end)],
            [6.0 * u * (1.0 - u) * (b - a) / seconds for a, b in zip(start, end)])


class TrajectoryAdapter(Node):
    def __init__(self, **kwargs):
        super().__init__('trajectory_adapter', **kwargs)
        self.declare_parameter('joint_names', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('robot_description', '')
        self.declare_parameter('command_timeout', 0.2)
        self.declare_parameter('feedback_timeout', 0.5)
        self.declare_parameter('home_timeout', 10.0)
        self.declare_parameter('max_joint_speed', 0.35)
        self.declare_parameter('input_topic', '/xr_device/input')

        self.timeout = float(self.get_parameter('command_timeout').value)
        self.feedback_timeout = float(self.get_parameter('feedback_timeout').value)
        self.home_timeout = float(self.get_parameter('home_timeout').value)
        self.max_speed = float(self.get_parameter('max_joint_speed').value)
        if min(self.timeout, self.feedback_timeout, self.home_timeout, self.max_speed) <= 0:
            raise ValueError('Adapter timeouts and speed must be positive')

        joint_names = list(self.get_parameter('joint_names').value)
        if len(joint_names) != 7:
            raise ValueError(f'Expected 7 joint names, got {len(joint_names)}: {joint_names}')
        self.joint_set = set(joint_names)

        root = ET.fromstring(self.get_parameter('robot_description').value)
        self.limits = {}
        for joint in root.findall('joint'):
            if joint.get('type') == 'revolute' and joint.get('name') in self.joint_set:
                limit = joint.find('limit')
                self.limits[joint.get('name')] = (
                    float(limit.get('lower')), float(limit.get('upper')))
        if len(self.limits) != 7:
            missing = self.joint_set - set(self.limits)
            raise ValueError(f'Missing URDF limits for joints: {missing}')

        self.feedback = {}
        self.feedback_at = -math.inf
        self.enabled = False
        self.pending = None
        self.command_at = -math.inf
        self.home = None

        self.pub = self.create_publisher(JointTrajectory, 'servo_joint_command', 1)
        self.driver_gate = self.create_client(SetBool, 'set_teleop_mode')
        self.gate_group = MutuallyExclusiveCallbackGroup()
        self.create_service(SetBool, 'control_enable', self.gate,
                            callback_group=self.gate_group)
        self.create_subscription(JointState, 'joint_states', self.state,
                                 qos_profile_sensor_data)
        self.create_subscription(JointTrajectory, 'joint_trajectory', self.command, 1)
        self.gripper_pub = self.create_publisher(Float32, 'gripper_command', 10)
        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(
            TeleopInput, self.get_parameter('input_topic').value,
            self.gripper_input, input_qos)
        self.create_timer(1.0 / 100.0, self.tick)

        ns = self.get_namespace()
        self.get_logger().info(
            f'AICO2 adapter [{ns}]: joint_trajectory -> servo_joint_command; '
            f'gripper -> gripper_command; gate control_enable active')

    def state(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(j in values and math.isfinite(values[j]) for j in self.limits):
            self.feedback = values
            self.feedback_at = time.monotonic()

    def gripper_input(self, msg):
        if not msg.gripper_valid:
            return
        out = Float32()
        out.data = float(msg.gripper)
        self.gripper_pub.publish(out)

    async def gate(self, request, response):
        if not request.data:
            self.enabled = False
            self.pending = self.home = None
        if not self.driver_gate.service_is_ready():
            response.success = False
            response.message = 'Driver is not configured'
            return response
        if request.data and time.monotonic() - self.feedback_at > self.feedback_timeout:
            response.success = False
            response.message = 'No fresh joint feedback'
            return response
        future = self.driver_gate.call_async(request)
        result = await future
        response.success = bool(result and result.success)
        response.message = result.message if result else 'Gate call failed'
        self.enabled = bool(request.data and response.success)
        if not self.enabled:
            self.pending = self.home = None
        return response

    def command(self, msg):
        if not msg.points:
            return
        names = list(msg.joint_names)
        point = msg.points[-1]
        if (len(names) != 7 or set(names) != self.joint_set
                or len(point.positions) != 7
                or (point.velocities and (len(point.velocities) != 7
                    or any(not math.isfinite(v) for v in point.velocities)))
                or any(not math.isfinite(q) or not self.limits[j][0] <= q <= self.limits[j][1]
                       for j, q in zip(names, point.positions))):
            self.get_logger().error('Rejected malformed or out-of-bounds command')
            return
        now = time.monotonic()
        if duration(point) > 0.5:
            if now - self.feedback_at > self.feedback_timeout:
                self.get_logger().error('Refused home without fresh joint feedback')
                return
            if len(msg.points) != 1:
                self.get_logger().error(
                    'Timed paths must use the driver FollowJointTrajectory action')
                return
            start = [self.feedback[j] for j in names]
            seconds = max(duration(point), max(1.5 * abs(b - a) / self.max_speed
                                               for a, b in zip(start, point.positions)))
            if seconds > self.home_timeout:
                self.get_logger().error('Refused home: duration exceeds home_timeout')
                return
            self.home = (names, start, list(point.positions), now, seconds)
        else:
            self.home = None
        self.pending = msg
        self.command_at = now

    def stop(self, reason):
        self.enabled = False
        self.pending = self.home = None
        self.get_logger().error(reason)
        if self.driver_gate.service_is_ready():
            request = SetBool.Request()
            request.data = False
            self.driver_gate.call_async(request)

    def tick(self):
        if not self.enabled:
            return
        now = time.monotonic()
        if now - self.feedback_at > self.feedback_timeout:
            self.stop('Feedback stale; stopping driver')
            return
        if self.pending is None:
            return
        if self.home is None:
            if now - self.command_at > self.timeout:
                self.stop('Command stream stale; stopping driver')
                return
            self.pub.publish(self.pending)
        else:
            names, start, target, began, seconds = self.home
            if now - began > seconds + 1.0:
                self.stop('Home exceeded its completion window')
                return
            q, dq = sample(start, target, now - began, seconds)
            msg = JointTrajectory()
            msg.joint_names = names
            point = JointTrajectoryPoint()
            point.positions, point.velocities = q, dq
            point.time_from_start.nanosec = 10000000
            msg.points = [point]
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop('Adapter shutting down')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
