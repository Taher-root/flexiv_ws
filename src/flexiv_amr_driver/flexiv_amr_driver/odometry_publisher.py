#!/usr/bin/env python3
"""
Odometry publisher using real wheel encoder velocity from Seer API 1005.

Instead of dead-reckoning from commanded velocity, this node queries the
chassis for actual measured velocity (r_vx, r_vy, r_w) derived from wheel
encoders, then integrates to produce odometry.

Published topics:
    /odom_raw (nav_msgs/Odometry) - wheel encoder odometry
    /amr/actual_velocity (geometry_msgs/Twist) - real-time measured velocity

TF broadcast:
    odom -> base_link (if publish_tf=true, but typically set false when EKF publishes TF)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import socket
import struct
import json
import math
import threading


class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        # Parameters
        self.declare_parameter('amr_ip', '192.168.1.110')
        self.declare_parameter('amr_port', 19204)
        self.declare_parameter('poll_rate', 50.0)  # Hz
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_tf', False)  # EKF publishes TF

        self.amr_ip = self.get_parameter('amr_ip').value
        self.amr_port = self.get_parameter('amr_port').value
        self.poll_rate = self.get_parameter('poll_rate').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Measured velocity from encoders
        self.real_vx = 0.0
        self.real_vy = 0.0
        self.real_w = 0.0
        self.is_stop = True

        # TCP connection
        self.sock = None
        self.sock_lock = threading.Lock()
        self.seq = 1

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)
        self.vel_pub = self.create_publisher(Twist, '/amr/actual_velocity', 10)

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Connect and start polling
        self._connect()
        self.poll_timer = self.create_timer(1.0 / self.poll_rate, self._poll_speed)
        self.odom_timer = self.create_timer(0.02, self._publish_odometry)  # 50Hz odom output

        self.get_logger().info(
            f'Encoder odometry publisher started (API 1005 @ {self.amr_ip}:{self.amr_port}, '
            f'poll_rate={self.poll_rate}Hz)')

    def _connect(self):
        """Establish TCP connection to Seer state port."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((self.amr_ip, self.amr_port))
            self.get_logger().info(f'Connected to Seer state port {self.amr_ip}:{self.amr_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            self.sock = None

    def _query_api(self, api_num):
        """Send API request and return parsed JSON response."""
        if self.sock is None:
            self._connect()
            if self.sock is None:
                return None

        try:
            with self.sock_lock:
                header = struct.pack('!BBHLH6s', 0x5A, 0x01, self.seq, 0, api_num, b'\x00' * 6)
                self.seq += 1
                self.sock.sendall(header)

                # Read response header
                resp_hdr = self.sock.recv(16)
                if len(resp_hdr) < 16:
                    return None
                _, _, _, json_len, _, _ = struct.unpack('!BBHLH6s', resp_hdr)

                if json_len == 0:
                    return {}

                # Read full JSON body
                body = b''
                while len(body) < json_len:
                    chunk = self.sock.recv(json_len - len(body))
                    if not chunk:
                        return None
                    body += chunk

                return json.loads(body.decode('utf-8'))
        except socket.timeout:
            self.get_logger().warn('API 1005 query timeout')
            return None
        except (ConnectionError, OSError) as e:
            self.get_logger().error(f'Connection lost: {e}, reconnecting...')
            self.sock = None
            return None

    def _poll_speed(self):
        """Query API 1005 for real encoder-derived velocity."""
        data = self._query_api(1005)
        if data is None:
            return

        # r_vx, r_vy, r_w = REAL measured velocity from wheel encoders
        self.real_vx = float(data.get('r_vx', 0.0))
        self.real_vy = float(data.get('r_vy', 0.0))
        self.real_w = float(data.get('r_w', 0.0))
        self.is_stop = bool(data.get('is_stop', True))

        # Publish actual velocity for other nodes
        vel_msg = Twist()
        vel_msg.linear.x = self.real_vx
        vel_msg.linear.y = self.real_vy
        vel_msg.angular.z = self.real_w
        self.vel_pub.publish(vel_msg)

    def _publish_odometry(self):
        """Integrate encoder velocity to produce odometry estimate."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0 or dt > 0.5:  # Skip if dt is unreasonable
            return

        # Integrate velocity in robot frame -> world frame
        delta_x = (self.real_vx * math.cos(self.theta) -
                   self.real_vy * math.sin(self.theta)) * dt
        delta_y = (self.real_vx * math.sin(self.theta) +
                   self.real_vy * math.cos(self.theta)) * dt
        delta_theta = self.real_w * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Build odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Covariance - position grows with distance (dead reckoning)
        odom.pose.covariance[0] = 0.1   # x
        odom.pose.covariance[7] = 0.1   # y
        odom.pose.covariance[35] = 0.05  # yaw

        # Velocity (direct from encoders - more trustworthy)
        odom.twist.twist.linear.x = self.real_vx
        odom.twist.twist.linear.y = self.real_vy
        odom.twist.twist.angular.z = self.real_w

        # Velocity covariance - tight, since these are encoder measurements
        odom.twist.covariance[0] = 0.01   # vx
        odom.twist.covariance[7] = 0.01   # vy
        odom.twist.covariance[35] = 0.02  # w

        self.odom_pub.publish(odom)

        # TF broadcast (only if EKF isn't doing it)
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
