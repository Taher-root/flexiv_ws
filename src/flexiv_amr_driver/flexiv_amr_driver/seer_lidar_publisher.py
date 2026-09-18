#!/usr/bin/env python3
"""Seer Robokit LiDAR publisher.

Queries Seer API 1009 (laser point cloud) via TCP and publishes
sensor_msgs/LaserScan for each LiDAR device.

Topics published:
  /scan/nav    - Front navigation LiDAR (device 'laser', id=0)
  /scan/avoid  - Rear obstacle-avoidance LiDAR (device 'laser1', id=1)
"""

from __future__ import annotations

import json
import math
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


SYNC_BYTE = 0x5A
PROTOCOL_VERSION = 0x01
API_LASER_POINT_CLOUD = 1009
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


class SeerLidarPublisher(Node):
    def __init__(self):
        super().__init__('seer_lidar_publisher')

        self.declare_parameter('amr_ip', '192.168.1.110')
        self.declare_parameter('state_port', 19204)
        self.declare_parameter('poll_rate_hz', 15.0)
        self.declare_parameter('nav_frame_id', 'lidar_nav_link')
        self.declare_parameter('avoid_frame_id', 'lidar_avoid_link')
        self.declare_parameter('socket_timeout', 2.0)
        self.declare_parameter('reconnect_interval', 5.0)

        self.amr_ip = self.get_parameter('amr_ip').value
        self.state_port = self.get_parameter('state_port').value
        self.poll_rate = self.get_parameter('poll_rate_hz').value
        self.nav_frame = self.get_parameter('nav_frame_id').value
        self.avoid_frame = self.get_parameter('avoid_frame_id').value
        self.socket_timeout = self.get_parameter('socket_timeout').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value

        self.nav_pub = self.create_publisher(LaserScan, '/scan/nav', 10)
        self.avoid_pub = self.create_publisher(LaserScan, '/scan/avoid', 10)

        self.sock = None
        self.sock_lock = threading.Lock()
        self.seq = 0
        self.last_connect_attempt = 0.0

        self._connect()
        self.timer = self.create_timer(1.0 / self.poll_rate, self._poll_and_publish)
        self.get_logger().info(
            f'Seer LiDAR publisher started: {self.amr_ip}:{self.state_port} @ {self.poll_rate} Hz')

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
                request = pack_request(self.seq, API_LASER_POINT_CLOUD)
                self.sock.sendall(request)
                header_data = recv_exact(self.sock, HEADER_SIZE)
                _, _, _, json_len, _, _ = struct.unpack(HEADER_FMT, header_data)
                body = recv_exact(self.sock, json_len)

            result = json.loads(body)
            stamp = self.get_clock().now().to_msg()

            for laser in result.get('lasers', []):
                device_info = laser.get('device_info', {})
                beams = laser.get('beams', [])

                device_name = device_info.get('device_name', 'laser')
                min_angle_deg = device_info.get('min_angle', -90)
                max_angle_deg = device_info.get('max_angle', 90)
                step_deg = device_info.get('pub_step', 0.5)
                min_range = device_info.get('min_range', 0.001)
                max_range = device_info.get('max_range', 40.0)
                scan_freq = device_info.get('scan_freq', 30)

                if device_name == 'laser':
                    pub = self.nav_pub
                    frame_id = self.nav_frame
                elif device_name == 'laser1':
                    pub = self.avoid_pub
                    frame_id = self.avoid_frame
                else:
                    continue

                scan = LaserScan()
                scan.header.stamp = stamp
                scan.header.frame_id = frame_id
                scan.angle_min = math.radians(min_angle_deg)
                scan.angle_max = math.radians(max_angle_deg)
                scan.angle_increment = math.radians(step_deg)
                scan.time_increment = 0.0
                scan.scan_time = 1.0 / scan_freq
                scan.range_min = float(min_range)
                scan.range_max = float(max_range)

                num_beams = int((max_angle_deg - min_angle_deg) / step_deg) + 1
                ranges = [float('inf')] * num_beams
                intensities = [0.0] * num_beams

                for beam in beams:
                    angle_deg = beam.get('angle', 0.0)
                    dist = beam.get('dist', 0.0)
                    rssi = beam.get('rssi', 0.0)
                    valid = beam.get('valid', False)
                    idx = int(round((angle_deg - min_angle_deg) / step_deg))
                    if 0 <= idx < num_beams and valid:
                        ranges[idx] = float(dist)
                        intensities[idx] = float(rssi)

                scan.ranges = ranges
                scan.intensities = intensities
                pub.publish(scan)

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
    node = SeerLidarPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
