"""Republish a robot's TF under a prefix so a replay can coexist with a live arm.

robot_state_publisher in Jazzy has no frame_prefix parameter, and two publishers
emitting transforms for the SAME frame names do not merge -- they interleave,
and the resulting tree jitters between two robots in a way that looks like a
control fault rather than a TF fault.

So the replay's robot_state_publisher is remapped to a private topic, and this
relays it into /tf with every frame renamed. RViz's RobotModel display has a
'TF Prefix' property that matches, which is what makes a second robot show up
alongside the real one instead of on top of it.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage

STATIC_QOS = QoSProfile(
    depth=100,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class TfPrefixer(Node):

    def __init__(self):
        super().__init__('tf_prefixer')
        self.declare_parameter('prefix', 'replay/')
        self.declare_parameter('input_topic', '/replay/tf')
        self.declare_parameter('input_static_topic', '/replay/tf_static')
        # The frame the replay tree hangs off. Left unprefixed so the replay
        # robot appears in the same world as the live one rather than floating
        # in a disconnected tree that RViz cannot resolve.
        self.declare_parameter('root_frame', 'world')

        self.prefix = self.get_parameter('prefix').value
        self.root = self.get_parameter('root_frame').value

        self._tf = self.create_publisher(TFMessage, '/tf', 100)
        self._tf_static = self.create_publisher(
            TFMessage, '/tf_static', STATIC_QOS)
        self.create_subscription(
            TFMessage, self.get_parameter('input_topic').value,
            lambda m: self._relay(m, self._tf), 100)
        self.create_subscription(
            TFMessage, self.get_parameter('input_static_topic').value,
            lambda m: self._relay(m, self._tf_static), STATIC_QOS)

        self.get_logger().info(
            f"prefixing replay TF with '{self.prefix}' "
            f"(root '{self.root}' left as is)")

    def _rename(self, frame_id: str) -> str:
        # The root keeps its name so the two trees share a world; everything
        # below it is prefixed so nothing collides.
        if frame_id == self.root:
            return frame_id
        return f'{self.prefix}{frame_id}'

    def _relay(self, msg: TFMessage, publisher) -> None:
        out = TFMessage()
        for transform in msg.transforms:
            transform.header.frame_id = self._rename(transform.header.frame_id)
            transform.child_frame_id = self._rename(transform.child_frame_id)
            out.transforms.append(transform)
        if out.transforms:
            publisher.publish(out)


def main():
    rclpy.init()
    node = TfPrefixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
