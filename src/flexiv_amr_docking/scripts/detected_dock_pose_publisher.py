#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


class DetectedDockPosePublisher(Node):
    def __init__(self):
        super().__init__('detected_dock_pose_publisher')

        # TF listener setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher to /detected_dock_pose
        self.publisher = self.create_publisher(
            PoseStamped,
            'detected_dock_pose',
            10
        )

        # Run at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Frames
        self.tag_frame   = 'tag36h11:1'
        self.fixed_frame = 'odom'

        # Skip transforms older than this (sec). lookup_transform with Time()
        # returns the newest entry in the buffer even if it is seconds stale,
        # so the age has to be checked explicitly.
        self.max_age = 0.5

        self.get_logger().info('detected_dock_pose_publisher started!')

    def timer_callback(self):
        try:
            # Ask TF: where is the tag in odom frame?
            transform = self.tf_buffer.lookup_transform(
                self.fixed_frame,       # target frame (odom)
                self.tag_frame,         # source frame (tag)
                rclpy.time.Time()       # latest available
            )

            age = (self.get_clock().now() - rclpy.time.Time.from_msg(
                transform.header.stamp)).nanoseconds / 1e9
            if age > self.max_age:
                self.get_logger().warn(
                    f'stale tag transform: {age:.2f}s old, skipping',
                    throttle_duration_sec=2.0)
                return

            # Package as PoseStamped. Note: the raw tag orientation is
            # published here on purpose -- SimpleChargingDock applies the
            # optical->nav rotation itself via external_detection_rotation_*.
            pose = PoseStamped()
            pose.header.stamp    = transform.header.stamp   # real detection time
            pose.header.frame_id = self.fixed_frame
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation.x = transform.transform.rotation.x
            pose.pose.orientation.y = transform.transform.rotation.y
            pose.pose.orientation.z = transform.transform.rotation.z
            pose.pose.orientation.w = transform.transform.rotation.w
            self.publisher.publish(pose)

        except Exception as e:
            self.get_logger().warn(
                f'TF lookup {self.fixed_frame} <- {self.tag_frame} failed: {e}',
                throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = DetectedDockPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
