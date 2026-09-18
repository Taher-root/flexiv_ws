#!/usr/bin/env python3
"""Seer Robokit IMU publisher.

Queries Seer API 1014 (IMU data) via TCP and publishes sensor_msgs/Imu.

Topic published:
  /imu/chassis - Seer chassis IMU data (accelerometer + gyro + orientation)

Units confirmed empirically:
  - acc_x/y/z: m/s^2 (acc_z ~ 9.81 at rest confirms this)
  - rot_x/y/z: rad/s (small values at rest confirms this)
  - Orientation: quaternion (qw, qx, qy, qz)
"""

from __future__ import annotations

import json
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


SYNC_BYTE = 0x5A
PROTOCOL_VERSION = 0x01
API_IMU_DATA = 1014
HEADER_FMT = '!BBHLH6s'
HEADER_SIZE = 16
RESERVED = b'\x00\x00\x00\x00\x00\x00'


def pack_request(seq: int, api_id: int, payload: dict | None = None) -> bytes:
    body = b''
    if payload:
        body = json.dumps(payload).encode('ascii')
    header = struct.pack(HEADER_FMT, SYNC_BYTE, PROTOCOL_VERSION, seq,
                         len(body), api_id, RESERVED)
    return header + body


def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(min(4096, n - len(buf)))
        if not chunk:
            raise ConnectionError("Socket closed while receiving data")
        buf += chunk
    return buf


class SeerImuPublisher(Node):
    def __init__(self):
        super().__init__('seer_imu_publisher')

        self.declare_parameter('amr_ip', '192.168.1.110')
        self.declare_parameter('state_port', 19204)
        self.declare_parameter('poll_rate_hz', 50.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('socket_timeout', 2.0)
        self.declare_parameter('reconnect_interval', 5.0)

        self.amr_ip = self.get_parameter('amr_ip').value
        self.state_port = self.get_parameter('state_port').value
        self.poll_rate = self.get_parameter('poll_rate_hz').value
        self.frame_id = self.get_parameter('frame_id').value
        self.socket_timeout = self.get_parameter('socket_timeout').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value

        self.imu_pub = self.create_publisher(Imu, '/imu/chassis', 10)

        self.sock = None
        self.sock_lock = threading.Lock()
        self.seq = 0
        self.last_connect_attempt = 0.0

        self._connect()
        self.timer = self.create_timer(1.0 / self.poll_rate, self._poll_and_publish)
        self.get_logger().info(
            f'Seer IMU publisher started: {self.amr_ip}:{self.state_port} @ {self.poll_rate} Hz')

    def _connect(self) -> bool:
        now = time.monotonic()
        if now - self.last_connect_attempt < self.reconnect_interval:
            return False
        self.last_connect_attempt = now
        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.socket_timeout)
            self.sock.connect((self.amr_ip, self.state_port))
            self.get_logger().info(f'Connected to Seer at {self.amr_ip}:{self.state_port}')
            return True
        except Exception as e:
            self.get_logger().warn(f'Failed to connect to Seer: {e}', throttle_duration_sec=10.0)
            self.sock = None
            return False

    def _poll_and_publish(self):
        if self.sock is None:
            self._connect()
            return
        try:
            with self.sock_lock:
                self.seq += 1
                request = pack_request(self.seq, API_IMU_DATA)
                self.sock.sendall(request)
                header_data = recv_exact(self.sock, HEADER_SIZE)
                _, _, _, json_len, _, _ = struct.unpack(HEADER_FMT, header_data)
                body = recv_exact(self.sock, json_len)

            result = json.loads(body)

            if result.get('ret_code', -1) != 0:
                self.get_logger().warn(
                    f'IMU API returned error: {result.get("ret_code")}',
                    throttle_duration_sec=10.0)
                return

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            # Orientation (quaternion)
            msg.orientation.w = result.get('qw', 1.0)
            msg.orientation.x = result.get('qx', 0.0)
            msg.orientation.y = result.get('qy', 0.0)
            msg.orientation.z = result.get('qz', 0.0)

            # Angular velocity (rad/s)
            msg.angular_velocity.x = result.get('rot_x', 0.0)
            msg.angular_velocity.y = result.get('rot_y', 0.0)
            msg.angular_velocity.z = result.get('rot_z', 0.0)

            # Linear acceleration (m/s^2)
            msg.linear_acceleration.x = result.get('acc_x', 0.0)
            msg.linear_acceleration.y = result.get('acc_y', 0.0)
            msg.linear_acceleration.z = result.get('acc_z', 0.0)

            # Covariance matrices for XvisioTech 6-DoF IMU (Seer chassis)
            # Based on industrial-grade 6-DoF IMU specifications:
            #   - Gyro noise density: ~0.008 °/s/√Hz → 1.0e-6 rad²/s² @ 50 Hz
            #   - Accel noise density: ~120 µg/√Hz → 8.0e-5 m²/s⁴ @ 50 Hz
            #   - Orientation drift: <1°/min → 8.0e-5 rad² short-term
            #
            # These values are ~1000x more accurate than previous placeholders (0.01, 0.001)
            # and reflect the high quality of the rigidly-mounted chassis IMU.
            #
            # Sensor Quality Ranking (best to worst):
            #   1. Seer Chassis IMU (XvisioTech 6-DoF) - BEST, rigidly mounted
            #   2. Wheel Encoders - Good for smooth surfaces
            #   3. RealSense D456 Camera IMUs (Bosch BMI085) - Consumer grade
            #   4. Visual Odometry - Depends on features/lighting

            # Orientation covariance (rad²) - Roll, Pitch, Yaw
            msg.orientation_covariance[0] = 8.0e-5  # roll (x)
            msg.orientation_covariance[4] = 8.0e-5  # pitch (y)
            msg.orientation_covariance[8] = 8.0e-5  # yaw (z)

            # Angular velocity covariance (rad²/s²) - X, Y, Z
            msg.angular_velocity_covariance[0] = 1.0e-6  # x
            msg.angular_velocity_covariance[4] = 1.0e-6  # y
            msg.angular_velocity_covariance[8] = 1.0e-6  # z

            # Linear acceleration covariance (m²/s⁴) - X, Y, Z
            msg.linear_acceleration_covariance[0] = 8.0e-5  # x
            msg.linear_acceleration_covariance[4] = 8.0e-5  # y
            msg.linear_acceleration_covariance[8] = 8.0e-5  # z

            self.imu_pub.publish(msg)

        except (ConnectionError, BrokenPipeError, OSError) as e:
            self.get_logger().warn(
                f'Connection lost: {e}. Reconnecting...', throttle_duration_sec=5.0
            )
            self.sock = None
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON decode error: {e}', throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}', throttle_duration_sec=5.0)

    def destroy_node(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SeerImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
