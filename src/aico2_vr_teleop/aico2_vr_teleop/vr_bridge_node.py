import os
import time
import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Float32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import ServoCommandType
from std_srvs.srv import SetBool

import uvicorn
from fastapi import FastAPI, Request
from starlette.responses import Response, FileResponse
from fastapi.staticfiles import StaticFiles

READY_POSITIONS = [0.0, -0.7145, 0.0, 1.6559, 0.0, 1.5708, 0.0]

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
        self.declare_parameter('gripper_force', 20.0)
        self.declare_parameter('hand', 'right')

        self.declare_parameter('planning_frame', 'Left_link0')
        self.declare_parameter('arm_ns', 'left_arm')
        self.declare_parameter('twist_topic', '/servo_node/delta_twist_cmds')

        self.declare_parameter('left_planning_frame', 'Left_link0')
        self.declare_parameter('left_arm_ns', 'left_arm')
        self.declare_parameter('left_twist_topic',
                               '/servo_node_left/delta_twist_cmds')
        self.declare_parameter('right_planning_frame', 'Right_link0')
        self.declare_parameter('right_arm_ns', 'right_arm')
        self.declare_parameter('right_twist_topic',
                               '/servo_node_right/delta_twist_cmds')

        self.declare_parameter('camera_topics', [
            '/cam_left/cam_left/color/image_raw',
            '/cam_right/cam_right/color/image_raw',
            '/cam_head/cam_head/color/image_raw',
        ])
        self.declare_parameter('camera_names', ['left', 'right', 'head'])

        port = int(self.get_parameter('port').value)
        host = str(self.get_parameter('host').value)
        self.max_lin = float(self.get_parameter('max_linear_vel').value)
        self.max_ang = float(self.get_parameter('max_angular_vel').value)
        self.clutch_thresh = float(self.get_parameter('clutch_threshold').value)
        self.gripper_force = float(self.get_parameter('gripper_force').value)
        hand_mode = str(self.get_parameter('hand').value)
        camera_topics = list(self.get_parameter('camera_topics').value)
        camera_names = list(self.get_parameter('camera_names').value)

        self._hands = {}
        self._lock = threading.Lock()

        if hand_mode == 'both':
            for side in ('left', 'right'):
                pf = str(self.get_parameter(f'{side}_planning_frame').value)
                ns = str(self.get_parameter(f'{side}_arm_ns').value)
                tt = str(self.get_parameter(f'{side}_twist_topic').value)
                prefix = ns.split('_')[0].capitalize()
                jnames = [f'{prefix}_joint{i}' for i in range(1, 8)]
                self._hands[side] = {
                    'planning_frame': pf,
                    'arm_ns': ns,
                    'twist_topic': tt,
                    'twist_pub': self.create_publisher(TwistStamped, tt, 10),
                    'gripper_pub': self.create_publisher(
                        Float32, f'/{ns}/gripper_command', 10),
                    'servo_cmd_pub': self.create_publisher(
                        JointTrajectory, f'/{ns}/servo_joint_command', 10),
                    'joint_names': jnames,
                    'gripper_grasping': False,
                    'prev_pose': None,
                    'prev_time': None,
                    'go_to_ready': False,
                    'button_a_held': False,
                    'button_b_held': False,
                }
        else:
            pf = str(self.get_parameter('planning_frame').value)
            ns = str(self.get_parameter('arm_ns').value)
            tt = str(self.get_parameter('twist_topic').value)
            prefix = ns.split('_')[0].capitalize()
            jnames = [f'{prefix}_joint{i}' for i in range(1, 8)]
            self._hands[hand_mode] = {
                'planning_frame': pf,
                'arm_ns': ns,
                'twist_topic': tt,
                'twist_pub': self.create_publisher(TwistStamped, tt, 10),
                'gripper_pub': self.create_publisher(
                    Float32, f'/{ns}/gripper_command', 10),
                'servo_cmd_pub': self.create_publisher(
                    JointTrajectory, f'/{ns}/servo_joint_command', 10),
                'joint_names': jnames,
                'gripper_grasping': False,
                'prev_pose': None,
                'prev_time': None,
                'go_to_ready': False,
                'button_a_held': False,
                'button_b_held': False,
            }

        self._record_toggle_pub = self.create_publisher(
            Empty, '/episode_recorder/toggle', 10)

        self.camera_jpgs = {}
        self._setup_cameras(camera_topics, camera_names)

        for _side, h in self._hands.items():
            self._switch_servo_to_twist(h['twist_topic'])
            self._enable_teleop_mode(h['arm_ns'])

        app = self._create_app()

        thread = threading.Thread(
            target=self._run_server, args=(app, host, port), daemon=True,
        )
        thread.start()
        hands_str = ', '.join(self._hands.keys())
        self.get_logger().info(
            f'VR bridge serving on http://{host}:{port} (hands: {hands_str})')

    # ------------------------------------------------------------------
    # Servo activation
    # ------------------------------------------------------------------

    def _switch_servo_to_twist(self, twist_topic):
        servo_prefix = twist_topic.rsplit('/delta_twist_cmds', 1)[0]
        srv_name = f'{servo_prefix}/switch_command_type'
        cli = self.create_client(ServoCommandType, srv_name)
        if not cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(
                f'{srv_name} not available; '
                'servo may not accept twist commands',
            )
            return
        req = ServoCommandType.Request()
        req.command_type = req.TWIST
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None:
            self.get_logger().info(f'Servo switched to TWIST ({srv_name})')
        else:
            self.get_logger().warn(f'Failed to switch servo to TWIST ({srv_name})')

    # ------------------------------------------------------------------
    # Teleop mode activation
    # ------------------------------------------------------------------

    def _enable_teleop_mode(self, arm_ns):
        srv_name = f'/{arm_ns}/set_teleop_mode'
        cli = self.create_client(SetBool, srv_name)
        if not cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(
                f'{srv_name} not available; '
                'rt_bridge may reject servo commands',
            )
            return
        for attempt in range(3):
            req = SetBool.Request()
            req.data = True
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            result = future.result()
            if result is not None and result.success:
                self.get_logger().info(
                    f'Teleop mode enabled ({arm_ns}): {result.message}')
                return
            msg = result.message if result is not None else 'no response'
            self.get_logger().warn(
                f'set_teleop_mode attempt {attempt + 1}/3 failed '
                f'({arm_ns}): {msg}',
            )
            time.sleep(2.0)
        self.get_logger().error(
            f'Could not enable teleop mode ({arm_ns}) after 3 attempts; '
            'arm will not move. Check that rt_server is running.',
        )

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
                items = data if isinstance(data, list) else [data]
                for d in items:
                    node._handle_pose(d)
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
        hand = message.get('hand', 'right')
        if hand not in self._hands:
            return

        h = self._hands[hand]

        front_trigger = float(message.get('triggerValue', 0))
        self._handle_gripper(h, front_trigger)

        button_a = float(message.get('buttonAValue', 0))
        button_b = float(message.get('buttonBValue', 0))
        back_trigger = float(message.get('backTriggerValue', 0))

        if button_b > 0.5 and not h['button_b_held']:
            h['button_b_held'] = True
            self._record_toggle_pub.publish(Empty())
        elif button_b < 0.2:
            h['button_b_held'] = False

        with self._lock:
            if button_a > 0.5 and not h['button_a_held']:
                h['button_a_held'] = True
                h['go_to_ready'] = not h['go_to_ready']
                if h['go_to_ready']:
                    h['prev_pose'] = None
                    h['prev_time'] = None
                    self._publish_twist(h, np.zeros(3), np.zeros(3))
                    self.get_logger().info(
                        f'Go-to-ready activated ({hand})')
                else:
                    self.get_logger().info(
                        f'Go-to-ready cancelled ({hand})')
            elif button_a < 0.2:
                h['button_a_held'] = False

            if h['go_to_ready']:
                self._publish_ready(h)
                return

        position = message['position']
        orientation = message['orientation']

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
                h['prev_pose'] = None
                h['prev_time'] = None
                self._publish_twist(h, np.zeros(3), np.zeros(3))
                return

            if h['prev_pose'] is None:
                h['prev_pose'] = pose_ros.copy()
                h['prev_time'] = now
                return

            dt = now - h['prev_time']
            if dt < 1e-4:
                return

            delta_pos = pose_ros[:3, 3] - h['prev_pose'][:3, 3]
            R_delta = pose_ros[:3, :3] @ h['prev_pose'][:3, :3].T

            lin_vel = delta_pos / dt
            twist_lin = np.clip(lin_vel / self.max_lin, -1.0, 1.0)

            axis, angle = _mat2axangle(R_delta)
            if abs(angle) < 1e-6:
                twist_ang = np.zeros(3)
            else:
                ang_vel = axis * angle / dt
                twist_ang = np.clip(ang_vel / self.max_ang, -1.0, 1.0)

            self._publish_twist(h, twist_lin, twist_ang)

            h['prev_pose'] = pose_ros.copy()
            h['prev_time'] = now

    def _handle_gripper(self, h, trigger):
        if trigger > 0.5 and not h['gripper_grasping']:
            h['gripper_grasping'] = True
            msg = Float32()
            msg.data = self.gripper_force
            h['gripper_pub'].publish(msg)
        elif trigger < 0.2 and h['gripper_grasping']:
            h['gripper_grasping'] = False
            msg = Float32()
            msg.data = 0.0
            h['gripper_pub'].publish(msg)

    def _publish_ready(self, h):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = h['joint_names']
        pt = JointTrajectoryPoint()
        pt.positions = READY_POSITIONS
        msg.points = [pt]
        h['servo_cmd_pub'].publish(msg)

    def _publish_twist(self, h, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = h['planning_frame']
        msg.twist.linear.x = float(linear[0])
        msg.twist.linear.y = float(linear[1])
        msg.twist.linear.z = float(linear[2])
        msg.twist.angular.x = float(angular[0])
        msg.twist.angular.y = float(angular[1])
        msg.twist.angular.z = float(angular[2])
        h['twist_pub'].publish(msg)

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
