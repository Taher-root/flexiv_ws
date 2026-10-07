import os
import json
import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Empty


class EpisodeRecorder(Node):

    def __init__(self):
        super().__init__('episode_recorder')

        self.declare_parameter('output_dir', '~/recordings')
        self.declare_parameter('record_rate', 15.0)
        self.declare_parameter('arm_ns', 'left_arm')
        self.declare_parameter('camera_topics', [
            '/cam_left/cam_left/color/image_raw',
            '/cam_right/cam_right/color/image_raw',
            '/cam_head/cam_head/color/image_raw',
        ])
        self.declare_parameter('camera_names', [
            'cam_left', 'cam_right', 'cam_head',
        ])
        self.declare_parameter('task_description', '')

        self.output_dir = os.path.expanduser(
            str(self.get_parameter('output_dir').value))
        self.record_rate = float(self.get_parameter('record_rate').value)
        arm_ns = str(self.get_parameter('arm_ns').value)
        camera_topics = list(self.get_parameter('camera_topics').value)
        self._camera_names = list(self.get_parameter('camera_names').value)
        self.task_description = str(
            self.get_parameter('task_description').value)

        os.makedirs(self.output_dir, exist_ok=True)

        self._lock = threading.Lock()
        self._recording = False
        self._latest_joints = None
        self._latest_gripper = 0.0
        self._latest_images = {}

        self._episode_states = []
        self._episode_timestamps = []
        self._episode_images = {}
        self._episode_index = self._next_episode_index()

        try:
            from cv_bridge import CvBridge
            import cv2
            self._cv_bridge = CvBridge()
            self._cv2 = cv2
        except ImportError:
            self.get_logger().error(
                'cv_bridge / cv2 required for episode recording')
            raise

        self.create_subscription(
            JointState, f'/{arm_ns}/joint_states',
            self._joint_state_cb, 10)
        self.create_subscription(
            JointState, f'/{arm_ns}/gripper_states',
            self._gripper_state_cb, 10)

        for topic, name in zip(camera_topics, self._camera_names):
            self.create_subscription(
                Image, topic,
                lambda msg, n=name: self._image_cb(msg, n), 1)

        self.create_subscription(
            Empty, '/episode_recorder/toggle',
            self._toggle_cb, 10)

        self._timer = self.create_timer(
            1.0 / self.record_rate, self._record_tick)

        self.get_logger().info(
            f'Episode recorder ready — {self.output_dir}, '
            f'{self.record_rate} Hz, next episode: {self._episode_index}')

    def _next_episode_index(self):
        if not os.path.isdir(self.output_dir):
            return 0
        indices = []
        for name in os.listdir(self.output_dir):
            if name.startswith('episode_'):
                try:
                    indices.append(int(name.split('_')[1]))
                except (ValueError, IndexError):
                    pass
        return max(indices) + 1 if indices else 0

    def _joint_state_cb(self, msg):
        arm_pos = []
        for name, pos in zip(msg.name, msg.position):
            if 'joint' in name.lower() and 'agv' not in name.lower():
                arm_pos.append(pos)
        if len(arm_pos) == 7:
            with self._lock:
                self._latest_joints = arm_pos

    def _gripper_state_cb(self, msg):
        if msg.position:
            with self._lock:
                self._latest_gripper = msg.position[0]

    def _image_cb(self, msg, name):
        try:
            cv_img = self._cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            _, jpg = self._cv2.imencode(
                '.jpg', cv_img,
                [self._cv2.IMWRITE_JPEG_QUALITY, 95])
            with self._lock:
                self._latest_images[name] = bytes(jpg)
        except Exception:
            pass

    def _toggle_cb(self, _msg):
        with self._lock:
            if self._recording:
                self._stop_recording()
            else:
                self._start_recording()

    def _start_recording(self):
        self._recording = True
        self._episode_states = []
        self._episode_timestamps = []
        self._episode_images = {n: [] for n in self._camera_names}
        self.get_logger().info(
            f'REC START — episode {self._episode_index}')

    def _stop_recording(self):
        self._recording = False
        if len(self._episode_states) < 2:
            self.get_logger().warn('Episode too short, discarded')
            return
        self._save_episode()
        self._episode_index += 1

    def _record_tick(self):
        if not self._recording:
            return
        with self._lock:
            if self._latest_joints is None:
                return
            state = self._latest_joints + [self._latest_gripper]
            self._episode_states.append(state)
            self._episode_timestamps.append(time.time())
            for name in self._camera_names:
                self._episode_images[name].append(
                    self._latest_images.get(name))

    def _save_episode(self):
        ep_dir = os.path.join(
            self.output_dir, f'episode_{self._episode_index:04d}')
        os.makedirs(ep_dir, exist_ok=True)

        for cam_name in self._camera_names:
            cam_dir = os.path.join(ep_dir, cam_name)
            os.makedirs(cam_dir, exist_ok=True)
            for i, jpg in enumerate(self._episode_images[cam_name]):
                if jpg is not None:
                    with open(os.path.join(cam_dir, f'{i:06d}.jpg'),
                              'wb') as f:
                        f.write(jpg)

        states = np.array(self._episode_states, dtype=np.float64)
        timestamps = np.array(self._episode_timestamps, dtype=np.float64)

        actions = np.empty_like(states)
        actions[:-1] = states[1:]
        actions[-1] = states[-1]

        np.save(os.path.join(ep_dir, 'states.npy'), states)
        np.save(os.path.join(ep_dir, 'actions.npy'), actions)
        np.save(os.path.join(ep_dir, 'timestamps.npy'), timestamps)

        meta = {
            'episode_index': self._episode_index,
            'num_frames': len(self._episode_states),
            'fps': self.record_rate,
            'cameras': self._camera_names,
            'state_dim': 8,
            'action_dim': 8,
            'state_keys': [
                'joint1', 'joint2', 'joint3', 'joint4',
                'joint5', 'joint6', 'joint7', 'gripper_width',
            ],
            'task': self.task_description,
            'duration_s': float(timestamps[-1] - timestamps[0]),
        }
        with open(os.path.join(ep_dir, 'metadata.json'), 'w') as f:
            json.dump(meta, f, indent=2)

        n = len(self._episode_states)
        dur = timestamps[-1] - timestamps[0]
        self.get_logger().info(
            f'Episode {self._episode_index} saved: '
            f'{n} frames, {dur:.1f}s -> {ep_dir}')


def main(args=None):
    rclpy.init(args=args)
    node = EpisodeRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
