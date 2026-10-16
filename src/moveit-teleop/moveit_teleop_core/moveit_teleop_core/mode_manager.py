"""The mode manager: state machine, deadman watchdog, and arbitration.

This node owns the things that break when they are scattered:

* **The deadman watchdog.** If operator input goes stale, the arm stops. The
  subtlety that makes this worth a dedicated owner: *ceasing to publish is not
  a stop*. In POSE mode servo keeps driving toward the last target it accepted,
  so a dead device leaves the arm sailing on. The watchdog must actively pause
  servo, and it must key on the device's own ``header.stamp`` rather than
  message arrival, because a driver that keeps republishing a frozen pose looks
  perfectly healthy to an arrival-time check.

* **Servo vs move_group arbitration.** Both drive the same
  JointTrajectoryController -- servo through its topic interface, move_group
  through its action interface -- and they preempt each other. TELEOP and
  PLANNING are separate states precisely so the two are never live at once.

* **The hardware gate.** On the real arm, the vendored ``agx_arm_control_gate``
  only opens the CAN gate during FollowJointTrajectory action goals, which
  never fire for servo output. Driving ``control_enable`` from here instead
  makes it a genuine deadman on the hardware path.

Concurrency note: every service call from a timer callback is fired
asynchronously and never waited on. ``spin_until_future_complete`` inside a
timer callback deadlocks a single-threaded executor, and a watchdog that
deadlocks is worse than no watchdog at all.
"""

from __future__ import annotations

import rclpy
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_teleop_msgs.msg import TeleopEvent, TeleopInput, TeleopStatus
from moveit_teleop_msgs.srv import SetTeleopMode

from .controllers import controller_for_joints, fetch_controller_map
from .planner import (
    NamedMoveClient, error_name, no_fallback_reason, should_fall_back,
)
from .robot_info import RobotInfo, fetch_descriptions
from .session import check_servo_model_matches, switch_servo_command_type

def gate_open_for(mode, move_via, open_for_planning: bool) -> bool:
    """Should the hardware gate be open, given what is driving the arm?

    The gate follows the COMMAND PATH, not the mode, because the two kinds of
    gate real drivers advertise behave oppositely during a planned move:

    * a **motion enable** (the Piper's CAN gate) must be open for anything to
      move at all, including a trajectory move_group executes through the
      controller's action interface.
    * a **streaming-mode select** (the Flexiv driver's set_teleop_mode) puts
      the arm in a servo mode that its own FollowJointTrajectory action
      refuses to run alongside -- it rejects the goal outright -- so it must
      be CLOSED for a planned move and open for a streamed one.

    ``open_for_planning`` picks between them, and only for trajectory
    execution. Streaming is unambiguous: teleoperation and the direct homing
    fallback both reach the arm by publishing to the controller topic, so both
    need the gate open whatever kind it is.
    """
    if mode == TeleopStatus.TELEOP:
        return True
    if mode != TeleopStatus.PLANNING:
        return False
    if move_via == 'direct':
        # The fallback streams to the controller topic exactly as teleop does.
        return True
    # A planned trajectory, executing through the controller's action.
    return bool(open_for_planning)


INPUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST, depth=1,
    durability=DurabilityPolicy.VOLATILE)

EVENT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST, depth=10,
    durability=DurabilityPolicy.VOLATILE)

MODE_NAMES = {
    TeleopStatus.IDLE: 'IDLE',
    TeleopStatus.ALIGNED: 'ALIGNED',
    TeleopStatus.TELEOP: 'TELEOP',
    TeleopStatus.PLANNING: 'PLANNING',
    TeleopStatus.HALTED: 'HALTED',
}

#: Servo status codes that mean the arm has stopped itself.
SERVO_HALT_CODES = {ServoStatus.HALT_FOR_SINGULARITY, ServoStatus.HALT_FOR_COLLISION}


class ModeManager(Node):

    def __init__(self):
        super().__init__('mode_manager')

        self.declare_parameter('move_group', 'arm')
        self.declare_parameter('ready_state', 'ready_pose')
        self.declare_parameter('description_source', '/move_group')
        self.declare_parameter('servo_node', '/servo_node')
        # Which node turns pose targets into joint commands.
        #   'servo' - moveit_servo: differential IK, halts a joint at its bound
        #   'ik'    - moveit_teleop_ik: full IK, limits as constraints
        # They differ in one respect that matters here: servo needs its command
        # type switched to POSE before it will act, and the IK solver has no
        # such notion.
        self.declare_parameter('command_source', 'servo')
        self.declare_parameter('joint_states_topic', '/control/joint_states')
        self.declare_parameter('input_topic', '/fake_device/input')
        self.declare_parameter('event_topic', '/fake_device/event')
        self.declare_parameter('controller', '')
        self.declare_parameter('goto_ready', True)

        self.declare_parameter('input_timeout', 0.1)
        self.declare_parameter('watchdog_rate', 50.0)
        self.declare_parameter('status_rate', 20.0)
        # Homing goes through move_group's MoveGroup action, so the path is
        # collision-checked and time-parameterised against the robot's own
        # limits. The direct publish below is the FALLBACK, and it is what this
        # node did exclusively before: one trajectory point at home_duration,
        # interpolated by the controller through whatever is in the way.
        #
        # The fallback needs nothing robot-specific. It publishes the same
        # trajectory_msgs/JointTrajectory to the same controller topic that
        # ik_node streams at 90 Hz, so an arm that can be teleoperated at all
        # can be homed this way.
        self.declare_parameter('home_via_moveit', True)
        self.declare_parameter('home_fallback', True)
        self.declare_parameter('move_action', '/move_action')
        self.declare_parameter('home_planning_time', 5.0)
        self.declare_parameter('home_planning_attempts', 2)
        # Fraction of the model's velocity AND acceleration limits. Homing is
        # the largest motion the arm makes and nobody is driving it.
        self.declare_parameter('home_scaling', 0.3)
        self.declare_parameter('home_duration', 3.0)
        self.declare_parameter('home_timeout', 12.0)
        self.declare_parameter('home_tolerance', 0.05)
        # Planning before the scene is applied is collision-aware against an
        # empty world, which is worse than not planning: it looks safe. Set
        # empty when no scene is configured, and nothing is waited for.
        self.declare_parameter('scene_ready_topic', '')
        self.declare_parameter('scene_ready_timeout', 10.0)

        # Hardware gate: off by default because in simulation there is no
        # driver advertising the service, and a manager that logs an error
        # every tick is a manager people turn off.
        self.declare_parameter('hardware_gate_enabled', False)
        self.declare_parameter('hardware_gate_service', '/control_enable')
        # Whether that gate must ALSO be open while move_group executes a
        # planned trajectory. True for a gate that enables motion; false for
        # one that selects a streaming mode the driver's own trajectory action
        # will not run alongside. See gate_open_for().
        self.declare_parameter('hardware_gate_open_for_planning', True)

        self.input_timeout = float(self.get_parameter('input_timeout').value)
        self.servo_node = self.get_parameter('servo_node').value
        self.command_source = str(self.get_parameter('command_source').value)
        if self.command_source not in ('servo', 'ik'):
            raise RuntimeError(
                f"command_source must be 'servo' or 'ik', got "
                f'{self.command_source!r}')

        # ---- robot model --------------------------------------------------
        source = self.get_parameter('description_source').value
        urdf, srdf = fetch_descriptions(self, source)
        self.robot = RobotInfo.from_xml(urdf, srdf)
        self.group = self.get_parameter('move_group').value
        joints = self.robot.group_joints(self.group)

        controller = self.get_parameter('controller').value
        if not controller:
            controller = controller_for_joints(
                fetch_controller_map(self, source), joints)
        self.controller = controller

        # ---- startup: reach a servoable posture, then enable POSE ----------
        # The MOTION is deferred to the first watchdog tick (see _goto_ready
        # below) rather than driven from here. Homing from the constructor
        # meant the one motion nobody is driving ran before the status
        # publisher, the watchdog and the e-stop existed, so it could be
        # neither observed nor interrupted -- and it was a second homing
        # implementation besides. The handshakes below move nothing, so they
        # stay here where blocking is safe.
        if self.command_source == 'servo':
            # Catch a servo/move_group model mismatch before it presents as a
            # missing service.
            check_servo_model_matches(self, self.servo_node, self.group, joints,
                                      reference=self.robot)
            switch_servo_command_type(self, self.servo_node)
        else:
            self.get_logger().info(
                f'command source is the full-IK solver at {self.servo_node}; '
                'no command-type switch needed')

        # ---- service clients (async only, from here on) -------------------
        pause_service = (f'{self.servo_node}/pause_servo'
                         if self.command_source == 'servo'
                         else f'{self.servo_node}/pause')
        self._pause_client = self.create_client(SetBool, pause_service)
        self._switch_client = self.create_client(
            ServoCommandType, f'{self.servo_node}/switch_command_type')
        self._gate_client = None
        if self.get_parameter('hardware_gate_enabled').value:
            self._gate_client = self.create_client(
                SetBool, self.get_parameter('hardware_gate_service').value)
            self.get_logger().info(
                f'hardware gate active on '
                f"'{self.get_parameter('hardware_gate_service').value}'")

        # ---- state --------------------------------------------------------
        self.mode = TeleopStatus.IDLE
        self.recording = False
        self.episode_id = ''
        self._last_input_stamp = None    # device sample time of the last fresh input
        self._clutch = False
        self._clutch_seen_released = True
        self._servo_status = -1
        self._servo_paused = None        # None = unknown, force a call on first tick
        self._gate_open = None
        self._message = 'starting'
        self._joint_state = {}
        self._home_request = False
        self._move_until = None
        self._move_via = None            # 'moveit' | 'direct', while homing
        self._move_result = None         # the planner's verdict, once it lands
        self._move_targets = {}
        self._move_joints = []
        self._move_label = ''
        self._move_fallback_ok = True
        self._waypoint = None            # a posture MARK_WAYPOINT remembered
        self._plan_and_go = False
        self.ready_state = self.get_parameter('ready_state').value
        self.home_duration = float(self.get_parameter('home_duration').value)
        self.home_timeout = float(self.get_parameter('home_timeout').value)
        self.home_tolerance = float(self.get_parameter('home_tolerance').value)
        self.home_fallback = bool(self.get_parameter('home_fallback').value)
        self.gate_for_planning = bool(
            self.get_parameter('hardware_gate_open_for_planning').value)
        self.group_joints = joints

        self.planner = None
        if bool(self.get_parameter('home_via_moveit').value):
            self.planner = NamedMoveClient(
                self, self.group,
                action_name=self.get_parameter('move_action').value,
                planning_time=float(
                    self.get_parameter('home_planning_time').value),
                attempts=int(
                    self.get_parameter('home_planning_attempts').value),
                scaling=float(self.get_parameter('home_scaling').value))

        self._scene_topic = str(self.get_parameter('scene_ready_topic').value)
        self._scene_ready = not self._scene_topic
        self._scene_deadline = self._now() + float(
            self.get_parameter('scene_ready_timeout').value)
        if self._scene_topic:
            # Latched by scene_objects, so this arrives even though that node
            # publishes once and this subscription may be created afterwards.
            self.create_subscription(
                Bool, self._scene_topic, self._on_scene_ready,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.create_subscription(
            TeleopInput, self.get_parameter('input_topic').value,
            self._on_input, INPUT_QOS)
        self.create_subscription(
            TeleopEvent, self.get_parameter('event_topic').value,
            self._on_event, EVENT_QOS)
        if self.command_source == 'servo':
            self.create_subscription(
                ServoStatus, f'{self.servo_node}/status',
                self._on_servo_status, 10)
        self.create_subscription(
            JointState, self.get_parameter('joint_states_topic').value,
            self._on_joint_states, 10)
        self._traj_pub = self.create_publisher(
            JointTrajectory, f'{self.controller}/joint_trajectory', 10)

        self.status_pub = self.create_publisher(TeleopStatus, '~/status', 10)
        self.create_service(SetTeleopMode, '~/set_mode', self._on_set_mode)

        self.create_timer(1.0 / float(self.get_parameter('watchdog_rate').value),
                          self._tick)
        self.create_timer(1.0 / float(self.get_parameter('status_rate').value),
                          self._publish_status)

        self._enter(TeleopStatus.IDLE, 'waiting for operator input')
        self.get_logger().info(
            f'watchdog armed: input_timeout={self.input_timeout}s on '
            f"'{self.get_parameter('input_topic').value}'")
        if bool(self.get_parameter('goto_ready').value):
            self._home_request = True
            self.get_logger().info(
                f'homing to {self.ready_state!r} before teleop can engage')

    # ---- inputs -----------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_input(self, msg: TeleopInput) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Key on the DEVICE's stamp, not arrival. A driver republishing a
        # frozen pose, or a network path delivering old samples, is exactly
        # the failure this must catch -- and it is invisible to an
        # arrival-time check.
        if self._now() - stamp <= self.input_timeout:
            self._last_input_stamp = stamp
        self._clutch = bool(msg.clutch) and bool(msg.pose_valid)
        if not self._clutch:
            self._clutch_seen_released = True

    def _on_event(self, msg: TeleopEvent) -> None:
        if msg.type == TeleopEvent.ESTOP:
            if self.planner is not None and self.planner.in_flight:
                # Stopping the solver does nothing to a trajectory move_group
                # is already executing through the controller.
                self.planner.cancel()
            self._enter(TeleopStatus.HALTED, 'e-stop from operator')
        elif msg.type == TeleopEvent.MARK_WAYPOINT:
            if not self._joint_state:
                self.get_logger().warn(
                    'MARK_WAYPOINT ignored: no joint state received yet')
            else:
                self._waypoint = {j: self._joint_state[j]
                                  for j in self.group_joints
                                  if j in self._joint_state}
                self.get_logger().info(
                    f'waypoint marked ({len(self._waypoint)} joints); '
                    'PLAN_AND_GO returns here')
        elif msg.type == TeleopEvent.PLAN_AND_GO:
            if self._waypoint is None:
                self.get_logger().warn(
                    'PLAN_AND_GO ignored: no waypoint marked')
            elif self._clutch_held():
                # Same reason HOME refuses: this move crosses the workspace
                # under the planner's control, not the operator's.
                self.get_logger().warn(
                    'PLAN_AND_GO ignored: release the clutch first')
            else:
                self._plan_and_go = True
                self.get_logger().info('PLAN_AND_GO requested')
        elif msg.type == TeleopEvent.HOME:
            if self._clutch_held():
                # Refuse while engaged: homing yanks the arm across the
                # workspace, and doing that under a held clutch is how someone
                # gets hit.
                self.get_logger().warn(
                    'HOME ignored: release the clutch first')
            else:
                self._home_request = True
                self.get_logger().info('HOME requested')
        elif msg.type == TeleopEvent.RECORD_START:
            self.recording = True
            self.episode_id = msg.payload or f'episode_{int(self._now())}'
            self.get_logger().info(f"recording started: '{self.episode_id}'")
        elif msg.type in (TeleopEvent.RECORD_STOP, TeleopEvent.EPISODE_SAVE):
            if self.recording:
                self.get_logger().info(f"recording stopped: '{self.episode_id}'")
            self.recording = False
        elif msg.type == TeleopEvent.EPISODE_DISCARD:
            self.recording = False
            self.get_logger().info(f"episode discarded: '{self.episode_id}'")

    def _on_servo_status(self, msg: ServoStatus) -> None:
        self._servo_status = int(msg.code)

    def _on_joint_states(self, msg: JointState) -> None:
        self._joint_state.update(zip(msg.name, msg.position))

    # ---- homing -----------------------------------------------------------

    def _on_scene_ready(self, msg: Bool) -> None:
        if msg.data and not self._scene_ready:
            self.get_logger().info(
                'planning scene applied; homing can be planned against it')
        self._scene_ready = self._scene_ready or bool(msg.data)

    def _ready_to_home(self) -> bool:
        """Hold the request back until homing can be planned properly.

        Only the scene is waited for, and only briefly. Planning before
        scene_objects has applied the table is collision-aware against an empty
        world, which is worse than not planning at all because it looks safe.

        No mode change while waiting: this is called from the watchdog at 50 Hz
        and a transition per tick would be 500 log lines in a 10 s wait.
        """
        if self.planner is None or self._scene_ready:
            return True
        if self._now() < self._scene_deadline:
            self.get_logger().info(
                f'homing waits for the planning scene on {self._scene_topic}',
                throttle_duration_sec=2.0)
            return False
        self.get_logger().warn(
            f'planning scene never reported ready on {self._scene_topic}; '
            'homing will be planned against whatever move_group holds now')
        self._scene_ready = True
        return True

    def _start_homing(self) -> None:
        """Resolve the ready posture and move there."""
        try:
            targets = self.robot.named_state(self.group, self.ready_state)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'cannot home: {exc}')
            self._home_request = False
            return
        self._home_request = False
        self._start_planned_move(targets, f'homing to {self.ready_state!r}',
                                 fallback_ok=self.home_fallback)

    def _start_planned_move(self, targets: dict, label: str,
                            fallback_ok: bool) -> None:
        """Move the arm to a joint configuration nobody is driving it to.

        Planned through move_group when it can be, because these are the only
        motions here with no collision guard of their own: a single trajectory
        point is interpolated by the controller straight through anything
        between the two postures.

        ``fallback_ok`` is per-move, not global. Homing may fall back to that
        interpolation, because its destination is a posture the robot's own
        SRDF calls safe and because refusing would mean an arm that will not
        start. An arbitrary remembered waypoint has neither excuse, so it does
        not fall back: if the path cannot be planned, nothing moves.

        Non-blocking either way. The planner reports through a callback and the
        direct publish is fire-and-forget; arrival is judged on later ticks.
        Spinning here would re-enter the executor and deadlock the watchdog.
        """
        self._move_targets = dict(targets)
        self._move_joints = [j for j in self.group_joints if j in targets]
        self._move_result = None
        self._move_label = label
        self._move_fallback_ok = bool(fallback_ok)
        self._move_until = self._now() + self.home_timeout
        # PLANNING pauses the solver before anything goes out, so the two are
        # never fighting over the controller.
        self._enter(TeleopStatus.PLANNING, label)

        if self._plan_move(self._move_targets):
            return
        if not self._move_fallback_ok:
            self._move_until = None
            self._enter(TeleopStatus.HALTED,
                        f'{label}: no planned path available, and this move '
                        'does not fall back to blind interpolation')
            return
        self._publish_move_trajectory(reason='planning unavailable')

    def _plan_move(self, targets: dict) -> bool:
        """Ask move_group to plan and execute. False if it cannot be asked."""
        if self.planner is None:
            return False
        if not self.planner.server_ready():
            self.get_logger().warn(
                f'no {self.get_parameter("move_action").value} action server: '
                'homing without a planned, collision-checked path')
            return False
        if not self.planner.send(targets, self._on_move_result):
            return False
        self._move_via = 'moveit'
        self.get_logger().info(
            f'planning a collision-checked path to {self.ready_state!r}')
        return True

    def _on_move_result(self, code: int) -> None:
        """Planner verdict. Recorded only -- the tick owns every transition."""
        self._move_result = int(code)

    def _publish_move_trajectory(self, reason: str) -> None:
        """The fallback: one timed point, interpolated by the controller.

        Robot-agnostic by construction. This is the same message type on the
        same controller topic that ik_node streams at 90 Hz, so it needs no
        vendor adapter that live teleoperation does not already need.
        """
        traj = JointTrajectory()
        traj.joint_names = list(self._move_joints)
        point = JointTrajectoryPoint()
        point.positions = [self._move_targets[j] for j in self._move_joints]
        point.time_from_start.sec = int(self.home_duration)
        point.time_from_start.nanosec = int((self.home_duration % 1.0) * 1e9)
        traj.points = [point]

        self._move_via = 'direct'
        self._move_result = None
        self._move_until = self._now() + self.home_timeout
        self.get_logger().warn(
            f'homing by direct interpolation ({reason}): the path is NOT '
            'collision-checked and the acceleration limits are not applied')
        # Gate first: _move_via is now 'direct', so this opens a gate that a
        # planned attempt may have required to be shut.
        self._set_gate(gate_open_for(self.mode, self._move_via,
                                     self.gate_for_planning))
        self._traj_pub.publish(traj)

    def _move_verdict(self) -> bool:
        """Act on the planner's verdict. True when homing is over either way."""
        code = self._move_result
        self._move_result = None
        if code == 1:                                     # SUCCESS
            return True
        if should_fall_back(code) and self._move_fallback_ok:
            self._publish_move_trajectory(
                reason=f'move_group refused: {error_name(code)}')
            return False
        self._move_until = None
        self._move_via = None
        self._enter(TeleopStatus.HALTED,
                    f'{self._move_label} refused: {error_name(code)} -- '
                    f'{no_fallback_reason(code)}')
        return False

    def _move_arrived(self) -> bool:
        if not self._joint_state:
            return False
        return all(abs(self._joint_state.get(j, 1e9) - self._move_targets[j])
                   < self.home_tolerance for j in self._move_joints)

    # ---- watchdog and state machine ---------------------------------------

    def _clutch_held(self) -> bool:
        """Is the operator engaged RIGHT NOW?

        A clutch level is only meaningful while input is fresh. Holding the
        last received value across a dropout means a device that dies mid-grip
        leaves the manager believing the clutch is held forever -- which then
        refuses homing, in precisely the situation where homing is what the
        operator needs.
        """
        return self._clutch and self._input_age() <= self.input_timeout

    def _input_age(self) -> float:
        if self._last_input_stamp is None:
            return float('inf')
        return self._now() - self._last_input_stamp

    def _tick(self) -> None:
        age = self._input_age()
        healthy = age <= self.input_timeout

        # Homing runs from any state, including HALTED -- recovering a bad
        # pose is exactly when it is needed, so it must not require the arm to
        # be healthy first.
        if self._home_request and self.mode != TeleopStatus.PLANNING:
            if self._ready_to_home():
                self._start_homing()
            self._apply_outputs()
            return

        if self._plan_and_go and self.mode != TeleopStatus.PLANNING:
            self._plan_and_go = False
            # No fallback: an arbitrary remembered posture has none of homing's
            # excuses for being driven to blindly.
            self._start_planned_move(self._waypoint,
                                     'returning to the marked waypoint',
                                     fallback_ok=False)
            self._apply_outputs()
            return

        if self.mode == TeleopStatus.PLANNING and self._move_until is not None:
            if self._move_result is not None:
                if not self._move_verdict():
                    self._apply_outputs()
                    return
                self._move_until = None
                self._move_via = None
                self._enter(TeleopStatus.ALIGNED if healthy else TeleopStatus.IDLE,
                            f'{self._move_label}: done')
                self._apply_outputs()
                return
            if self._move_via == 'moveit':
                # The planner owns arrival on this path; the deadline below is
                # only a backstop for a result that never comes.
                if self._now() > self._move_until:
                    self._move_until = None
                    self._move_via = None
                    self.planner.cancel()
                    self._enter(TeleopStatus.HALTED,
                                f'{self._move_label}: no result from '
                                f'move_group within {self.home_timeout:.0f}s')
                self._apply_outputs()
                return
            if self._move_arrived():
                self._move_until = None
                self._enter(TeleopStatus.ALIGNED if healthy else TeleopStatus.IDLE,
                            f'homed to {self.ready_state!r}')
            elif self._now() > self._move_until:
                self._move_until = None
                self._enter(TeleopStatus.HALTED, 'homing timed out')
            self._apply_outputs()
            return

        if not healthy and self.mode in (TeleopStatus.TELEOP, TeleopStatus.ALIGNED):
            self._enter(TeleopStatus.HALTED,
                        f'input stale by {age:.3f}s (limit {self.input_timeout}s)')
        elif self._servo_status in SERVO_HALT_CODES and self.mode == TeleopStatus.TELEOP:
            self._enter(TeleopStatus.HALTED, f'servo halted (code {self._servo_status})')
        elif self.mode == TeleopStatus.HALTED:
            # Recovery demands a DELIBERATE re-engage: input healthy again AND
            # the clutch released at least once since the fault. Without the
            # release requirement, an operator still holding the clutch through
            # a dropout would have the arm resume under their hand.
            if healthy and self._clutch_seen_released and not self._clutch_held():
                self._enter(TeleopStatus.ALIGNED, 'recovered; clutch to engage')
        elif self.mode == TeleopStatus.IDLE:
            if healthy:
                self._enter(TeleopStatus.ALIGNED, 'input healthy; clutch to engage')
        elif self.mode == TeleopStatus.ALIGNED:
            if self._clutch_held():
                self._enter(TeleopStatus.TELEOP, 'engaged')
        elif self.mode == TeleopStatus.TELEOP:
            if not self._clutch_held():
                self._enter(TeleopStatus.ALIGNED, 'disengaged')

        self._apply_outputs()

    def _enter(self, mode: int, message: str) -> None:
        if mode == self.mode:
            self._message = message
            return
        self.get_logger().info(
            f'{MODE_NAMES.get(self.mode, self.mode)} -> '
            f'{MODE_NAMES.get(mode, mode)}: {message}')
        self.mode = mode
        self._message = message
        if mode == TeleopStatus.HALTED:
            # Require a fresh release before anything can re-engage.
            self._clutch_seen_released = False
        self._apply_outputs()

    def _apply_outputs(self) -> None:
        """Drive servo and the hardware gate to match the current mode.

        Only acts on change: re-sending the same request every tick would put
        a service call on the wire at watchdog rate for no reason.
        """
        should_run = self.mode == TeleopStatus.TELEOP
        self._set_servo_paused(not should_run)
        self._set_gate(gate_open_for(self.mode, self._move_via,
                                     self.gate_for_planning))

    def _set_servo_paused(self, paused: bool) -> None:
        if self._servo_paused == paused:
            return
        if not self._pause_client.service_is_ready():
            return
        self._servo_paused = paused
        request = SetBool.Request()
        request.data = paused
        self._pause_client.call_async(request)   # deliberately not awaited

    def _set_gate(self, open_: bool) -> None:
        if self._gate_client is None or self._gate_open == open_:
            return
        if not self._gate_client.service_is_ready():
            return
        self._gate_open = open_
        request = SetBool.Request()
        request.data = open_
        self._gate_client.call_async(request)

    # ---- services and status ----------------------------------------------

    def _on_set_mode(self, request, response):
        target = request.mode
        if target not in MODE_NAMES:
            response.success = False
            response.message = f'unknown mode {target}'
            return response
        if target in (TeleopStatus.TELEOP, TeleopStatus.ALIGNED) and \
                self._input_age() > self.input_timeout:
            response.success = False
            response.message = (
                f'refused: input stale by {self._input_age():.3f}s. '
                'Entering a live mode without a healthy operator link is '
                'exactly what the watchdog exists to prevent.')
            return response
        self._enter(target, 'requested via set_mode')
        response.success = True
        response.message = f'mode is {MODE_NAMES[target]}'
        return response

    def _publish_status(self) -> None:
        msg = TeleopStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = self.mode
        msg.recording = self.recording
        msg.episode_id = self.episode_id
        age = self._input_age()
        msg.input_healthy = age <= self.input_timeout
        msg.input_age = float(min(age, 1e6))
        msg.servo_status = int(self._servo_status)
        msg.message = self._message
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ModeManager()
    except Exception as exc:  # noqa: BLE001
        rclpy.logging.get_logger('mode_manager').fatal(str(exc))
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
