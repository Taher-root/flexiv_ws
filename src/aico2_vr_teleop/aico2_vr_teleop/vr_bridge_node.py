import os
import time
import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image

import uvicorn
from fastapi import FastAPI, Request
from starlette.responses import Response, FileResponse
from fastapi.staticfiles import StaticFiles

TF_RUB2FLU = np.array([
    [0, 0, -1, 0],
    [-1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 0, 1],
])


def _quat2mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])


def _pose_matrix(pos, quat):
    T = np.eye(4)
    T[:3, :3] = _quat2mat(quat)
    T[:3, 3] = pos
    return T


def _mat2axangle(R):
    cos_angle = (np.trace(R) - 1.0) / 2.0
    angle = math.acos(max(-1.0, min(1.0, cos_angle)))
    if abs(angle) < 1e-6:
        return np.array([0.0, 0.0, 1.0]), 0.0
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ]) / (2.0 * math.sin(angle))
    norm = np.linalg.norm(axis)
    if norm < 1e-9:
        return np.array([0.0, 0.0, 1.0]), 0.0
    return axis / norm, angle


class VRBridgeNode(Node):

    def __init__(self):
        super().__init__('vr_bridge')

        self.declare_parameter('port', 8181)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('max_linear_vel', 0.3)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('clutch_threshold', 0.95)
        self.declare_parameter('planning_frame', 'Left_link0')
        self.declare_parameter('camera_topics', [
            '/cam_left/cam_left/color/image_raw',
            '/cam_right/cam_right/color/image_raw',
        ])
        self.declare_parameter('camera_names', ['left', 'right'])

        port = int(self.get_parameter('port').value)
        host = str(self.get_parameter('host').value)
        self.max_lin = float(self.get_parameter('max_linear_vel').value)
        self.max_ang = float(self.get_parameter('max_angular_vel').value)
        self.clutch_thresh = float(self.get_parameter('clutch_threshold').value)
        self.planning_frame = str(self.get_parameter('planning_frame').value)
        camera_topics = list(self.get_parameter('camera_topics').value)
        camera_names = list(self.get_parameter('camera_names').value)

        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10,
        )

        self._prev_pose = None
        self._prev_time = None
        self._lock = threading.Lock()

        self.camera_jpgs = {}
        self._setup_cameras(camera_topics, camera_names)

        app = self._create_app()

        thread = threading.Thread(
            target=self._run_server, args=(app, host, port), daemon=True,
        )
        thread.start()
        self.get_logger().info(f'VR bridge serving on http://{host}:{port}')

    # ------------------------------------------------------------------
    # Camera subscriptions
    # ------------------------------------------------------------------

    def _setup_cameras(self, topics, names):
        try:
            from cv_bridge import CvBridge
            import cv2
            self._cv_bridge = CvBridge()
            self._cv2 = cv2
        except ImportError:
            self.get_logger().warn(
                'cv_bridge or cv2 not found; camera feeds disabled',
            )
            return

        for topic, name in zip(topics, names):
            self.create_subscription(
                Image, topic,
                lambda msg, n=name: self._camera_cb(msg, n),
                1,
            )
            self.get_logger().info(f'Camera "{name}": {topic}')

    def _camera_cb(self, msg, name):
        try:
            cv_img = self._cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            _, jpg = self._cv2.imencode(
                '.jpg', cv_img,
                [self._cv2.IMWRITE_JPEG_QUALITY, 60],
            )
            self.camera_jpgs[name] = bytes(jpg)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # FastAPI application
    # ------------------------------------------------------------------

    def _create_app(self):
        app = FastAPI()

        try:
            from ament_index_python.packages import get_package_share_directory
            webxr_dir = os.path.join(
                get_package_share_directory('aico2_vr_teleop'), 'webxr',
            )
        except Exception:
            webxr_dir = os.path.join(
                os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
                'webxr',
            )

        static_dir = os.path.join(webxr_dir, 'static')
        if os.path.isdir(static_dir):
            app.mount(
                '/static',
                StaticFiles(directory=static_dir),
                name='static',
            )

        node = self

        @app.get('/')
        def index():
            return FileResponse(os.path.join(webxr_dir, 'index.html'))

        @app.post('/pose')
        async def pose(request: Request):
            try:
                data = await request.json()
                node._handle_pose(data)
                status = 'ok'
            except Exception:
                status = 'error'
            return {'status': status}

        @app.get('/cam/{name}/snapshot')
        def cam_snapshot(name: str):
            jpg = node.camera_jpgs.get(name)
            if jpg is None:
                return Response(status_code=404)
            return Response(content=jpg, media_type='image/jpeg')

        @app.post('/log')
        async def log_msg(request: Request):
            try:
                data = await request.json()
                node.get_logger().info(f'VR: {data}')
            except Exception:
                pass
            return {'status': 'ok'}

        return app

    # ------------------------------------------------------------------
    # Pose handling — called from the FastAPI thread
    # ------------------------------------------------------------------

    def _handle_pose(self, message):
        position = message['position']
        orientation = message['orientation']
        back_trigger = float(message.get('backTriggerValue', 0))

        pos = np.array([position['x'], position['y'], position['z']])
        quat = np.array([
            orientation['w'], orientation['x'],
            orientation['y'], orientation['z'],
        ])

        pose_rub = _pose_matrix(pos, quat)
        pose_ros = TF_RUB2FLU @ pose_rub
        pose_ros[:3, :3] = (
            pose_ros[:3, :3] @ np.linalg.inv(TF_RUB2FLU[:3, :3])
        )

        now = time.monotonic()
        clutch = back_trigger > self.clutch_thresh

        with self._lock:
            if not clutch:
                self._prev_pose = None
                self._prev_time = None
                self._publish_twist(np.zeros(3), np.zeros(3))
                return

            if self._prev_pose is None:
                self._prev_pose = pose_ros.copy()
                self._prev_time = now
                return

            dt = now - self._prev_time
            if dt < 1e-4:
                return

            delta_pos = pose_ros[:3, 3] - self._prev_pose[:3, 3]
            R_delta = pose_ros[:3, :3] @ self._prev_pose[:3, :3].T

            lin_vel = delta_pos / dt
            twist_lin = np.clip(lin_vel / self.max_lin, -1.0, 1.0)

            axis, angle = _mat2axangle(R_delta)
            if abs(angle) < 1e-6:
                twist_ang = np.zeros(3)
            else:
                ang_vel = axis * angle / dt
                twist_ang = np.clip(ang_vel / self.max_ang, -1.0, 1.0)

            self._publish_twist(twist_lin, twist_ang)

            self._prev_pose = pose_ros.copy()
            self._prev_time = now

    def _publish_twist(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.planning_frame
        msg.twist.linear.x = float(linear[0])
        msg.twist.linear.y = float(linear[1])
        msg.twist.linear.z = float(linear[2])
        msg.twist.angular.x = float(angular[0])
        msg.twist.angular.y = float(angular[1])
        msg.twist.angular.z = float(angular[2])
        self.twist_pub.publish(msg)

    # ------------------------------------------------------------------
    # Server lifecycle
    # ------------------------------------------------------------------

    @staticmethod
    def _run_server(app, host, port):
        import logging
        for name in ('uvicorn', 'uvicorn.access', 'uvicorn.error'):
            logging.getLogger(name).setLevel(logging.WARNING)
        uvicorn.run(app, host=host, port=port, log_level='warning')


def main(args=None):
    rclpy.init(args=args)
    node = VRBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
