#!/usr/bin/env python3
"""Relay Quest thumbstick axes to waist joint commands and AMR velocity.

Right thumbstick  -> waist (AGV_Joint1/2) via velocity integration
Left thumbstick   -> AMR /cmd_vel (geometry_msgs/Twist)

The right stick deflection is treated as angular velocity: each tick the
waist target is incremented by stick * waist_speed * dt, then clipped to
the URDF joint limits.  Releasing the stick stops the waist where it is.

The left stick maps directly to linear.x (forward/back) and angular.z
(turn).  Releasing the stick sends zero velocity so the AMR stops.
"""
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray

WAIST_LIMITS = {
    'AGV_Joint1': (-1.5707, 1.5707),
    'AGV_Joint2': (0.0, 1.5707),
}


class VRJoystickRelay(Node):
    def __init__(self, **kwargs):
        super().__init__('vr_joystick_relay', **kwargs)
        self.declare_parameter('joy_topic', '/xr_device/thumbsticks')
        self.declare_parameter('waist_speed', 0.5)
        self.declare_parameter('amr_linear_speed', 0.3)
        self.declare_parameter('amr_angular_speed', 0.5)
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('rate', 50.0)

        self.waist_speed = float(self.get_parameter('waist_speed').value)
        self.amr_lin = float(self.get_parameter('amr_linear_speed').value)
        self.amr_ang = float(self.get_parameter('amr_angular_speed').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        rate = float(self.get_parameter('rate').value)

        self.waist_q = [0.0, 0.0]
        self.waist_initialized = False
        self.stick = [0.0, 0.0, 0.0, 0.0]
        self.last_tick = time.monotonic()

        self.create_subscription(
            Joy, self.get_parameter('joy_topic').value,
            self._joy_cb, 10)
        self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_cb, qos_profile_sensor_data)

        self.waist_pub = self.create_publisher(
            Float64MultiArray, '/left_arm/waist_override', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'VR joystick relay: right stick -> waist, left stick -> /cmd_vel '
            f'(waist_speed={self.waist_speed}, deadzone={self.deadzone})')

    def _dz(self, v):
        if abs(v) < self.deadzone:
            return 0.0
        sign = 1.0 if v > 0 else -1.0
        return sign * (abs(v) - self.deadzone) / (1.0 - self.deadzone)

    def _joy_cb(self, msg):
        if len(msg.axes) >= 4:
            self.stick = [float(msg.axes[i]) for i in range(4)]

    def _joint_state_cb(self, msg):
        if self.waist_initialized:
            return
        names = list(msg.name)
        positions = list(msg.position)
        if 'AGV_Joint1' in names and 'AGV_Joint2' in names:
            self.waist_q = [
                float(positions[names.index('AGV_Joint1')]),
                float(positions[names.index('AGV_Joint2')]),
            ]
            self.waist_initialized = True
            self.get_logger().info(
                f'Waist seeded from feedback: '
                f'[{self.waist_q[0]:.3f}, {self.waist_q[1]:.3f}]')

    def _tick(self):
        now = time.monotonic()
        dt = min(now - self.last_tick, 0.1)
        self.last_tick = now

        right_x = self._dz(self.stick[2])
        right_y = self._dz(self.stick[3])
        left_x = self._dz(self.stick[0])
        left_y = self._dz(self.stick[1])

        if self.waist_initialized:
            self.waist_q[0] += right_x * self.waist_speed * dt
            self.waist_q[1] += right_y * self.waist_speed * dt
            lo1, hi1 = WAIST_LIMITS['AGV_Joint1']
            lo2, hi2 = WAIST_LIMITS['AGV_Joint2']
            self.waist_q[0] = max(lo1, min(hi1, self.waist_q[0]))
            self.waist_q[1] = max(lo2, min(hi2, self.waist_q[1]))
            msg = Float64MultiArray()
            msg.data = list(self.waist_q)
            self.waist_pub.publish(msg)

        twist = Twist()
        twist.linear.x = left_y * self.amr_lin
        twist.angular.z = -left_x * self.amr_ang
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = VRJoystickRelay()
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
