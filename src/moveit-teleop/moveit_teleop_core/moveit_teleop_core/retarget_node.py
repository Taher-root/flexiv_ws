"""Retargeting node: normalized device input -> end-effector pose target.

In step 3 of the bring-up this node is deliberately NOT connected to servo. It
publishes a pose and, more usefully, broadcasts that pose as a TF frame so the
commanded target can be watched next to the real end effector in RViz.

That TF frame is the whole point. Coordinate-convention bugs are the single
largest time sink in teleoperation, and they are almost impossible to reason
about from numbers but obvious the moment two frames are drawn side by side.
Building the visualization before connecting the robot is the cheap ordering.

Robot-specific values are resolved from robot_description and
robot_description_semantic at startup -- no link, joint or frame name is
hardcoded here or required in config.
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from moveit_teleop_msgs.msg import (
    TeleopFeedback, TeleopInput, TeleopStatus,
)

from .retargeting.base import RetargetInput, load_retargeter
from .robot_info import RobotInfo, fetch_descriptions
from .scene_config import resolve_planes

#: Must match the device side; see moveit_teleop_devices.device_base.
INPUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST, depth=1,
    durability=DurabilityPolicy.VOLATILE)


class RetargetNode(Node):

    def __init__(self):
        super().__init__('retarget_node')

        self.declare_parameter('move_group', 'arm')
        self.declare_parameter('ee_link', '')          # '' -> derive from SRDF
        self.declare_parameter('planning_frame', '')   # '' -> URDF root link
        self.declare_parameter('description_source', '/move_group')
        self.declare_parameter('retargeter', 'relative_pose')
        self.declare_parameter('input_topic', '/fake_device/input')
        self.declare_parameter('target_frame', 'teleop_target')
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('status_topic', '/mode_manager/status')
        # Only accumulate while the mode manager says the system is live.
        # Anchoring during startup homing -- or any other motion we did not
        # command -- produces a target offset from reality that then drags the
        # arm into a joint limit. The external-motion guard is a backstop for
        # this, not a substitute: it can only react after the fact.
        self.declare_parameter('require_teleop_mode', True)

        self.declare_parameter('translation_scale', 0.5)
        self.declare_parameter('rotation_scale', 1.0)
        self.declare_parameter('translation_ref', 'base')
        self.declare_parameter('rotation_ref', 'tool')
        self.declare_parameter('device_convention', 'ros')
        self.declare_parameter('smoothing_alpha', 1.0)
        self.declare_parameter('max_translation_step', 0.05)
        self.declare_parameter('external_motion_threshold', 0.15)
        self.declare_parameter('max_target_lead', 0.05)
        self.declare_parameter('max_target_lead_angle', 0.5)
        # A wall appearing or disappearing under the target moves the
        # COMMAND without the hand having moved -- reach past the edge of
        # a table and the command snaps from the surface down to where the
        # hand actually is. Past this much unexplained motion the mapping
        # is re-zeroed and the arm holds, instead of teleporting.
        self.declare_parameter('max_command_step', 0.03)
        # The same planning-scene file the collision objects and the IK
        # solver read. Walls from it are projected onto here, BEFORE the
        # leash, so the target slides along a surface instead of pushing
        # through it and then snapping back.
        self.declare_parameter('scene_config', '')
        self.declare_parameter('feedback_topic', '/teleop/feedback')
        # Buzz strength at the surface and at full depth.
        self.declare_parameter('contact_depth_full', 0.05)

        self.move_group = self.get_parameter('move_group').value
        self.target_frame = self.get_parameter('target_frame').value

        # ---- resolve the robot from MoveIt's own artifacts ----------------
        source = self.get_parameter('description_source').value
        urdf, srdf = fetch_descriptions(self, source)
        self.robot = RobotInfo.from_xml(urdf, srdf)

        self.planning_frame = (self.get_parameter('planning_frame').value
                               or self.robot.model_frame)
        self.ee_link = (self.get_parameter('ee_link').value
                        or self.robot.end_effector_link(self.move_group))

        self.get_logger().info(
            f"group='{self.move_group}' ee_link='{self.ee_link}' "
            f"planning_frame='{self.planning_frame}' "
            f"(both resolved from the robot model, not config)")

        # ---- retargeter ---------------------------------------------------
        config = {
            'translation_scale': self.get_parameter('translation_scale').value,
            'rotation_scale': self.get_parameter('rotation_scale').value,
            'translation_ref': self.get_parameter('translation_ref').value,
            'rotation_ref': self.get_parameter('rotation_ref').value,
            'device_convention': self.get_parameter('device_convention').value,
            'max_translation_step': self.get_parameter('max_translation_step').value,
            'external_motion_threshold':
                self.get_parameter('external_motion_threshold').value,
            'max_target_lead': self.get_parameter('max_target_lead').value,
            'max_target_lead_angle':
                self.get_parameter('max_target_lead_angle').value,
            'max_command_step':
                self.get_parameter('max_command_step').value,
            'smoothing': {'alpha': self.get_parameter('smoothing_alpha').value},
        }
        name = self.get_parameter('retargeter').value
        self.retargeter = load_retargeter(name, config, self.robot)
        walls = resolve_planes(self, self.get_parameter('scene_config').value,
                               self.planning_frame)
        if hasattr(self.retargeter, 'walls'):
            self.retargeter.walls = walls
        for wall in walls:
            extent = ('unbounded' if wall.extent is None
                      else f'within {wall.extent.round(3).tolist()}')
            self.get_logger().info(
                f"wall '{wall.name}': {wall.normal.round(3).tolist()} . p "
                f'>= {wall.offset:.4f}, {extent} '
                '-- the target slides along it')
        self.get_logger().info(f"retargeter='{name}' config={config}")

        # ---- io -----------------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.target_pub = self.create_publisher(PoseStamped, '~/target_pose', 10)
        # Transient local: contact is a LEVEL, so a device that connects
        # midway through must be told the current state rather than
        # inferring 'no contact' from silence.
        self.feedback_pub = self.create_publisher(
            TeleopFeedback, self.get_parameter('feedback_topic').value,
            QoSProfile(depth=1,
                       history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._feedback_active = False
        # Announce the resting state once, so a device that connects late
        # knows there is no contact rather than inferring it from silence.
        self._feedback_announced = False
        self.create_subscription(
            TeleopInput, self.get_parameter('input_topic').value,
            self._on_input, INPUT_QOS)
        self.create_subscription(
            TeleopStatus, self.get_parameter('status_topic').value,
            self._on_status, 10)

        self._latest = None
        self._last_reason = None
        self._mode = None
        self.require_mode = bool(self.get_parameter('require_teleop_mode').value)
        self._warned_no_tf = False
        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"listening on '{self.get_parameter('input_topic').value}', "
            f"broadcasting TF '{self.planning_frame}' -> '{self.target_frame}'")

    def _on_status(self, msg: TeleopStatus) -> None:
        was_live = self._mode == TeleopStatus.TELEOP
        self._mode = msg.mode
        if was_live and msg.mode != TeleopStatus.TELEOP:
            # Whatever happens next -- homing, a planned move, a halt -- moves
            # the arm without us. Drop the anchor so the next engage takes a
            # fresh one from wherever the arm actually ends up.
            self.retargeter.reset()

    def _live(self) -> bool:
        if not self.require_mode:
            return True
        return self._mode == TeleopStatus.TELEOP

    def _on_input(self, msg: TeleopInput) -> None:
        # Only stash it. Retargeting runs on the timer so the output rate is
        # decoupled from a device's own (possibly jittery) rate.
        self._latest = msg

    def _current_ee_pose(self):
        """Look up the actual EE pose. Returns None if TF is not ready."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.planning_frame, self.ee_link, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001 - TF raises several types
            if not self._warned_no_tf:
                self.get_logger().warn(
                    f'no TF {self.planning_frame} -> {self.ee_link} yet: {exc}')
                self._warned_no_tf = True
            return None
        self._warned_no_tf = False
        t, r = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z]), np.array([r.x, r.y, r.z, r.w])

    def _tick(self) -> None:
        if self._latest is None:
            return
        pose = self._current_ee_pose()

        if not self._live():
            # Not engaged: hold the anchor released so nothing accumulates
            # against a pose the arm is being moved away from.
            self.retargeter.reset()
            # Keep the target frame ON the end effector while disengaged.
            #
            # Returning without broadcasting leaves 'teleop_target' frozen
            # wherever it last was, so during homing the arm drives away from a
            # stale marker and the RViz view suggests a huge tracking error
            # that does not exist. Following the EE is also the honest reading:
            # re-engaging anchors here, so this IS where the target would be.
            if pose is not None:
                self._broadcast(*pose)
            return

        if pose is None:
            return
        ee_position, ee_orientation = pose

        msg = self._latest
        sample = RetargetInput(
            device_position=np.array([msg.pose.position.x,
                                      msg.pose.position.y,
                                      msg.pose.position.z]),
            device_orientation=np.array([msg.pose.orientation.x,
                                         msg.pose.orientation.y,
                                         msg.pose.orientation.z,
                                         msg.pose.orientation.w]),
            clutch=msg.clutch,
            pose_valid=msg.pose_valid,
            gripper=msg.gripper,
            ee_position=ee_position,
            ee_orientation=ee_orientation,
        )

        out = self.retargeter.update(sample)

        if out.reason != self._last_reason:
            self.get_logger().info(f'retargeting: {out.reason}')
            self._last_reason = out.reason

        # Always broadcast the frame, active or not, so RViz shows where the
        # target sits even while declutched. Publish the POSE only when active,
        # because a pose message means "go here" to everything downstream.
        self._broadcast(out.position, out.orientation)
        if out.active:
            self._publish_pose(out.position, out.orientation)
        self._publish_feedback(out.active)

    def _publish_feedback(self, active: bool) -> None:
        """Tell the operator when they are pressing into a wall.

        Sent as a LEVEL: published on every edge and then held, so a device can
        run a continuous rumble and stop when it clears. Publishing only the
        rising edge would leave a device guessing how long to buzz, and a
        dropped message would leave it buzzing indefinitely.
        """
        contacts = list(getattr(self.retargeter, 'contacts', []) or [])
        touching = bool(active and contacts)
        depth = float(getattr(self.retargeter, 'contact_depth', 0.0) or 0.0)

        # Edge-triggered publishing, level semantics: resend only when the
        # state changes, plus a periodic refresh while active so a subscriber
        # that joined late is not left thinking nothing is happening.
        self._feedback_ticks = getattr(self, '_feedback_ticks', 0) + 1
        changed = touching != self._feedback_active
        if not self._feedback_announced:
            changed, self._feedback_announced = True, True
        if not changed and not (touching and self._feedback_ticks % 25 == 0):
            return
        self._feedback_active = touching

        msg = TeleopFeedback()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.active = touching
        msg.sources = contacts if touching else []
        msg.depth = depth if touching else 0.0
        if touching:
            full = float(self.get_parameter('contact_depth_full').value)
            # Ramp from a light touch at the surface to full at `full` metres
            # past it, so the operator can feel HOW hard they are pressing
            # rather than just that they are.
            msg.intensity = float(min(1.0, 0.25 + 0.75 * depth / max(full, 1e-6)))
            msg.duration = 0.25
        self.feedback_pub.publish(msg)

    def _broadcast(self, position, orientation) -> None:
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.planning_frame
        tf.child_frame_id = self.target_frame
        tf.transform.translation.x = float(position[0])
        tf.transform.translation.y = float(position[1])
        tf.transform.translation.z = float(position[2])
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = (
            float(v) for v in orientation)
        self.tf_broadcaster.sendTransform(tf)

    def _publish_pose(self, position, orientation) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.planning_frame
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = (
            float(v) for v in orientation)
        self.target_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RetargetNode()
    except Exception as exc:  # noqa: BLE001
        rclpy.logging.get_logger('retarget_node').fatal(str(exc))
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(1)
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
