"""Latch a string onto a topic, and keep latching it.

MoveIt's RobotState display -- the one that draws the translucent preview robot
-- builds its own RobotModel, which needs BOTH the URDF and the SRDF. The URDF
arrives from the replay's robot_state_publisher; nothing publishes a semantic
description outside a running move_group, and the whole point of the replay
launch is that it does not need one.

So the launch runs xacro on the SRDF and this latches the result. Trivial, but
without it the display logs 'Robot semantic description not found' and renders
nothing, which looks exactly like a broken episode rather than a missing topic.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class DescriptionPublisher(Node):

    def __init__(self):
        super().__init__('description_publisher')
        self.declare_parameter('topic', '/replay/robot_description_semantic')
        self.declare_parameter('content', '')

        topic = self.get_parameter('topic').value
        content = self.get_parameter('content').value
        if not content:
            self.get_logger().warn(
                f'no content given; {topic} will be published empty')

        self._publisher = self.create_publisher(String, topic, LATCHED)
        self._message = String(data=content)
        self._publisher.publish(self._message)
        self.get_logger().info(f'latched {len(content)} chars on {topic}')


def main():
    rclpy.init()
    node = DescriptionPublisher()
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
