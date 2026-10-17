#!/usr/bin/env python3
"""Single-screen bimanual XR bridge.

Runs ONE HTTPS/WebSocket server on one port. The Quest headset sends both
controllers in every frame; this node publishes each to its own TeleopInput
topic so the left and right teleop pipelines both receive input from a single
browser tab.

Usage (standalone):
    ros2 run aico2_teleop xr_bimanual_bridge.py --ros-args \
        -p certfile:=~/.config/moveit_teleop/xr_cert.pem \
        -p keyfile:=~/.config/moveit_teleop/xr_key.pem

Usually launched automatically by vr.launch.py with arm:=both.
"""
from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)

from moveit_teleop_msgs.msg import TeleopEvent, TeleopInput
from sensor_msgs.msg import Joy

from moveit_teleop_devices.xr_device import _Bridge

INPUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

EVENT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)


class XRBimanualBridge(Node):

    def __init__(self):
        super().__init__('xr_bimanual_bridge')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8443)
        self.declare_parameter('certfile', '')
        self.declare_parameter('keyfile', '')
        self.declare_parameter('rate_hz', 90.0)
        self.declare_parameter('gripper_deadzone', 0.03)
        self.declare_parameter('clutch_threshold', 0.5)
        self.declare_parameter('web_dir', '')
        self.declare_parameter('frame_id', 'webxr_local_floor')

        self._deadzone = float(self.get_parameter('gripper_deadzone').value)
        self._clutch_thresh = float(self.get_parameter('clutch_threshold').value)
        self._frame_id = str(self.get_parameter('frame_id').value)

        from pathlib import Path
        from ament_index_python.packages import get_package_share_directory
        web_dir = self.get_parameter('web_dir').value or str(
            Path(get_package_share_directory('moveit_teleop_devices')) / 'web')

        certfile = self.get_parameter('certfile').value
        keyfile = self.get_parameter('keyfile').value
        if not certfile or not keyfile:
            raise RuntimeError(
                'certfile and keyfile are required for HTTPS.')

        self._left_pub = self.create_publisher(
            TeleopInput, 'left/input', INPUT_QOS)
        self._right_pub = self.create_publisher(
            TeleopInput, 'right/input', INPUT_QOS)
        self._left_event_pub = self.create_publisher(
            TeleopEvent, 'left/event', EVENT_QOS)
        self._right_event_pub = self.create_publisher(
            TeleopEvent, 'right/event', EVENT_QOS)
        self._joy_pub = self.create_publisher(Joy, 'thumbsticks', 10)

        self._bridge = _Bridge(
            self.get_parameter('host').value,
            int(self.get_parameter('port').value),
            certfile, keyfile, web_dir, self.get_logger())
        self._bridge.start()

        rate_hz = float(self.get_parameter('rate_hz').value)
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

        self._last_headset_t = None
        self._prev = {'left': {}, 'right': {}}
        self._recording = {'left': False, 'right': False}

        self.get_logger().info(
            f'bimanual XR bridge publishing at {rate_hz:.0f} Hz on '
            f'{self._left_pub.topic_name} and {self._right_pub.topic_name}')

    def _tick(self):
        frame = self._bridge.take()
        if frame is None:
            return

        headset_t = frame.get('t')
        if headset_t is None or headset_t == self._last_headset_t:
            return
        self._last_headset_t = headset_t

        stamp = self.get_clock().now().to_msg()
        all_controllers = frame.get('controllers') or {}

        left_c = all_controllers.get('left')
        right_c = all_controllers.get('right')
        joy_msg = Joy()
        joy_msg.header.stamp = stamp
        joy_msg.axes = [
            float(left_c.get('sx', 0.0)) if left_c else 0.0,
            float(left_c.get('sy', 0.0)) if left_c else 0.0,
            float(right_c.get('sx', 0.0)) if right_c else 0.0,
            float(right_c.get('sy', 0.0)) if right_c else 0.0,
        ]
        self._joy_pub.publish(joy_msg)

        for hand, pub, event_pub in [
            ('left', self._left_pub, self._left_event_pub),
            ('right', self._right_pub, self._right_event_pub),
        ]:
            controller = all_controllers.get(hand)
            if controller is None:
                continue
            self._publish_hand(hand, controller, pub, event_pub, stamp)

    def _publish_hand(self, hand, controller, pub, event_pub, stamp):
        msg = TeleopInput()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id

        valid = bool(controller.get('valid', False))
        msg.pose_valid = valid

        if valid:
            p = controller.get('p', (0.0, 0.0, 0.0))
            q = controller.get('q', (0.0, 0.0, 0.0, 1.0))
            msg.pose.position.x = float(p[0])
            msg.pose.position.y = float(p[1])
            msg.pose.position.z = float(p[2])
            msg.pose.orientation.x = float(q[0])
            msg.pose.orientation.y = float(q[1])
            msg.pose.orientation.z = float(q[2])
            msg.pose.orientation.w = float(q[3])

        squeeze = float(controller.get('squeeze', 0.0))
        msg.clutch = squeeze >= self._clutch_thresh

        trigger = float(controller.get('trigger', 0.0))
        msg.gripper = 0.0 if trigger < self._deadzone else trigger
        msg.gripper_valid = True

        pub.publish(msg)

        for name, handler in (('a', '_on_a'), ('b', '_on_b'),
                              ('stick', '_on_stick')):
            pressed = bool(controller.get(name, False))
            if pressed and not self._prev[hand].get(name, False):
                events = []
                getattr(self, handler)(hand, events)
                for event_type, payload in events:
                    evt = TeleopEvent()
                    evt.header.stamp = stamp
                    evt.header.frame_id = self._frame_id
                    evt.type = int(event_type)
                    evt.payload = payload
                    event_pub.publish(evt)
            self._prev[hand][name] = pressed

    def _on_a(self, hand, events):
        self._recording[hand] = not self._recording[hand]
        if self._recording[hand]:
            events.append((TeleopEvent.RECORD_START, ''))
            self.get_logger().info(f'{hand} A: recording STARTED')
        else:
            events.append((TeleopEvent.RECORD_STOP, ''))
            events.append((TeleopEvent.EPISODE_SAVE, ''))
            self.get_logger().info(f'{hand} A: recording STOPPED and saved')

    def _on_stick(self, hand, events):
        events.append((TeleopEvent.HOME, ''))
        self.get_logger().info(f'{hand} thumbstick: HOME')

    def _on_b(self, hand, events):
        if self._recording[hand]:
            self._recording[hand] = False
        events.append((TeleopEvent.EPISODE_DISCARD, ''))
        self.get_logger().info(f'{hand} B: discard last episode')


def main(args=None):
    rclpy.init(args=args)
    node = XRBimanualBridge()
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
