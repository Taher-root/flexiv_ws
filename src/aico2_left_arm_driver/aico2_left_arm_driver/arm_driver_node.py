#!/usr/bin/env python3
"""Lifecycle node for one Flexiv Rizon arm (mock or flexivrdk 1.9)."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from aico2_msgs.msg import ArmStatus
from aico2_msgs.srv import ExecutePrimitive
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory

from aico2_grippers.flexiv_gripper import FlexivGripper
from aico2_grippers.force_grasp_logic import ForceGraspController, ForceGraspParams
from aico2_left_arm_driver.cartesian_stream import integrate_tcp_pose
from aico2_left_arm_driver.flexiv_session import (
    FlexivSession,
    import_flexivrdk,
    mode_to_str,
    tcp_pose_to_ros,
    wrench6_to_ros,
)
from aico2_left_arm_driver.offset_estimator import OffsetTracker
from aico2_left_arm_driver.ros_time import seconds_to_ros_time
from aico2_left_arm_driver.trajectory_stream import (
    reorder_to_driver,
    sample_trajectory,
    trajectory_duration,
)

_STALE_WARN_INTERVAL_SEC = 2.0


def _namespace_defaults(namespace: str) -> tuple[str, List[str]]:
    """Fallback joint names / TCP frame when YAML params are not applied."""
    if "right" in namespace.lower():
        joints = [f"Right_joint{i}" for i in range(1, 8)]
        return "Right_flange", joints
    joints = [f"Left_joint{i}" for i in range(1, 8)]
    return "Left_flange", joints


@dataclass(frozen=True)
class _ArmSample:
    """One RDK reading for the *publishing* path only.

    Replaced whole, never mutated, so the poll thread and the publish timer
    share it without a lock (joint_state_architecture.md sec 4.1: whole-object
    attribute assignment is atomic under the GIL).

    The control path (trajectory streaming, servo, Cartesian) deliberately
    does NOT read from here — it calls states() directly at command time,
    where it needs the freshest possible value rather than the last decimated
    sample.
    """

    device_timestamp: Tuple[int, int]  # RobotStates.timestamp: (sec, nanosec)
    host_mono: float
    q: List[float]
    dq: List[float]
    tau: List[float]
    tcp_pose: List[float]
    ext_wrench: List[float]


class ArmDriverNode(LifecycleNode):
    """Publishes arm state; streams NRT joint trajectories when not in mock mode."""

    def __init__(self, node_name: str, namespace: str) -> None:
        super().__init__(node_name, namespace=namespace)
        self._namespace = namespace
        self._default_tcp, self._default_joints = _namespace_defaults(namespace)
        self._declare_parameters()
        self._joint_names: List[str] = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        if not self._joint_names:
            self._joint_names = list(self._default_joints)
        self._validate_joint_names()
        self._mock = bool(self.get_parameter("mock_hardware").value)
        self._rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._robot_sn = str(self.get_parameter("robot_sn").value).strip()
        self._auto_enable = bool(self.get_parameter("auto_enable").value)
        self._op_timeout = float(self.get_parameter("operational_timeout_sec").value)
        self._dof = len(self._joint_names)
        self._rdk_dof = self._dof
        self._arm_dof = self._dof
        self._ext_dof = 0
        self._mock_idle_q = self._float_list_param(
            "mock_idle_joint_positions", 0.0, self._dof
        )
        # After mock trajectories finish, hold last commanded pose (not idle home).
        self._mock_hold_q = list(self._mock_idle_q)
        self._max_vel = self._float_list_param(
            "default_max_joint_vel", 1.5, self._dof
        )
        self._max_acc = self._float_list_param(
            "default_max_joint_acc", 3.0, self._dof
        )
        self._goal_tol = float(self.get_parameter("goal_joint_tolerance").value)
        self._tcp_frame = str(self.get_parameter("tcp_frame_id").value)
        # Filled in on_configure when launch YAML params are fully applied.
        self._teleop_backend = "servo"
        self._joint_control_mode = "position"
        self._joint_stiffness_ratio = 1.0
        self._goal_settle_timeout = 3.0
        self._goal_tol_impedance = 0.05
        self._max_contact_torque = 0.0
        self._traj_send_rate = 50.0
        self._cartesian_max_lin = 0.3
        self._cartesian_max_ang = 1.047
        self._use_device_timestamp = False
        self._offset_refresh_sec = 300.0
        self._offset_slew_limit = 0.001
        self._stale_threshold_sec = 0.05
        self._publish_effort = True

        self._js_pub = None
        self._status_pub = None
        self._tcp_pub = None
        self._wrench_pub = None
        self._fault_pub = None
        self._mode_pub = None
        self._timer = None
        self._t0 = 0.0
        self._session: Optional[FlexivSession] = None
        self._hw_ready = False

        # Publishing path: free-running poll thread -> latest-value slot,
        # decoupled from the publish timer (sec 4.1/4.3). Control path is
        # untouched and still reads states() directly.
        self._sample: Optional[_ArmSample] = None
        self._poll_thread: Optional[threading.Thread] = None
        self._poll_stop = threading.Event()
        self._offset: Optional[OffsetTracker] = None

        self._exec_prim_srv = None
        self._clear_fault_srv = None
        self._traj_server = None
        self._cb_group = ReentrantCallbackGroup()

        self._active_traj: Optional[JointTrajectory] = None
        self._traj_t0: float = 0.0
        self._traj_goal_handle = None

        # Teleop (MoveIt Servo) streaming input. Mutually exclusive with the
        # FollowJointTrajectory action via the _teleop_active latch.
        self._teleop_active = False
        self._servo_sub = None
        self._set_teleop_srv = None
        self._servo_prev_q: Optional[List[float]] = None
        self._servo_prev_time: Optional[float] = None
        self._servo_mode_retry_t: float = 0.0

        self._cartesian_pose: Optional[List[float]] = None
        self._cartesian_prev_t: Optional[float] = None
        self._cartesian_sub = None

        self._log_rdk_send_rate = bool(
            self.get_parameter("log_rdk_send_rate").value
        )
        self._log_rdk_send_interval = float(
            self.get_parameter("log_rdk_send_interval_sec").value
        )
        self._rdk_send_stats: dict[str, dict] = {}

        self._gripper_enabled = False
        self._gripper_name = "Flexiv-GN01"
        self._gripper: Optional[FlexivGripper] = None
        self._gripper_ctrl: Optional[ForceGraspController] = None
        self._gripper_sub = None
        self._gripper_width_pub = None

    def _declare_parameters(self) -> None:
        is_right = "right" in self._namespace.lower()
        self.declare_parameter("robot_ip", "192.168.1.101" if is_right else "192.168.1.100")
        self.declare_parameter("robot_sn", "")
        self.declare_parameter("mock_hardware", True)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("auto_enable", True)
        self.declare_parameter("operational_timeout_sec", 30.0)
        self.declare_parameter("default_max_joint_vel", 1.5)
        self.declare_parameter("default_max_joint_acc", 3.0)
        self.declare_parameter("goal_joint_tolerance", 0.02)
        self.declare_parameter(
            "teleop_backend",
            "servo",
        )  # "servo" (MoveIt joint stream) or "cartesian" (RDK SendCartesianMotionForce)
        self.declare_parameter("cartesian_max_linear_vel", 0.3)
        self.declare_parameter("cartesian_max_angular_vel", 1.047)  # ~60 deg/s
        self.declare_parameter("log_rdk_send_rate", True)
        self.declare_parameter("log_rdk_send_interval_sec", 1.0)
        self.declare_parameter("gripper_enabled", False)
        self.declare_parameter("gripper_name", "Flexiv-GN01")
        self.declare_parameter("gripper_max_force_fraction", 0.75)
        self.declare_parameter("gripper_open_threshold", 0.08)
        self.declare_parameter("gripper_grasp_threshold", 0.15)
        self.declare_parameter("gripper_force_resend_delta", 3.0)
        self.declare_parameter("tcp_frame_id", self._default_tcp)
        self.declare_parameter("mock_idle_joint_positions", [0.0] * 7)
        self.declare_parameter("joint_names", self._default_joints)
        # Timestamping (sec 4.3/4.4). Defaults to false here, unlike
        # aico2_waist_driver: the arms already have live TF consumers, so
        # enabling device timestamps is opt-in per arm rather than a silent
        # change on upgrade. Flip to true once verified against the other,
        # untouched arm as a control (sec 8 step 4).
        self.declare_parameter("use_device_timestamp", False)
        self.declare_parameter("offset_refresh_sec", 300.0)
        self.declare_parameter("offset_slew_limit", 0.001)
        self.declare_parameter("stale_threshold_sec", 0.05)
        self.declare_parameter("publish_effort", True)
        # Joint control mode for trajectories and servo teleop. "position" is
        # NRT_JOINT_POSITION, the historical behaviour. "impedance" is
        # NRT_JOINT_IMPEDANCE, which tracks the same SendJointPosition stream
        # (RDK lists both modes as applicable) but lets the arm yield to
        # external force by joint_stiffness_ratio * K_q_nom. Default stays
        # "position" so this is opt-in.
        self.declare_parameter("joint_control_mode", "position")
        self.declare_parameter("joint_stiffness_ratio", 1.0)
        # How long past a trajectory's own duration to keep waiting for the arm
        # to settle inside goal_joint_tolerance. Without a bound the execute
        # loop spins until the client gives up, and the client's timeout
        # (MoveIt's allowed_execution_duration_scaling) reports a generic
        # CONTROL_FAILED while the driver, which knows the residual error,
        # says nothing. Keep this under the client's own limit so the driver
        # is the one that explains the failure.
        self.declare_parameter("goal_settle_timeout_sec", 3.0)
        # A compliant arm cannot hold position as tightly as a stiff one: under
        # joint impedance the steady-state error is (unmodelled torque / K_q),
        # which on this robot is dominated by the undeclared gripper mass and
        # measured around 1.7 deg at joint 4. That exceeds
        # goal_joint_tolerance (0.02 rad = 1.15 deg), so convergence could never
        # be declared and every trajectory ran until the client gave up.
        # Impedance mode therefore gets its own, wider tolerance. Tightening it
        # is a matter of declaring the tool in Flexiv Elements, not of tuning
        # this number.
        self.declare_parameter("goal_joint_tolerance_impedance", 0.05)
        # Torque ceiling per arm axis in impedance mode. Without it the
        # impedance law demands stiffness x deflection with no bound and the
        # joint simply saturates: at nominal K_q, arm joint 4 (4200 Nm/rad)
        # demands 294 Nm for 4 degrees of deflection against its 64 Nm limit,
        # so a blocked arm pushes with everything it has until the controller's
        # collision detection trips. SetMaxContactTorque makes it yield at a
        # chosen torque instead. 0 or less leaves it unset (previous
        # behaviour); the URDF's own effort limits are 123/123/64/64/39/39/39.
        self.declare_parameter("max_contact_torque", 0.0)
        # Rate at which trajectory samples are pushed to the controller. Was
        # welded to publish_rate_hz, which conflated two unrelated things: how
        # often state is published, and how often the arm is re-commanded.
        #
        # It matters for smoothness. RDK's SendJointPosition re-plans inside the
        # controller on every call, aborting the previous plan, so a high send
        # rate means the internal motion generator never finishes a segment --
        # at 50 Hz it is restarted every 20 ms. Sending fewer, farther-apart
        # setpoints and letting the generator interpolate is often smoother,
        # which is counterintuitive but follows directly from the re-planning
        # behaviour. 0 or less means "use publish_rate_hz", the old behaviour.
        self.declare_parameter("trajectory_send_rate_hz", 0.0)

    def _validate_joint_names(self) -> None:
        """Catch mis-loaded params (e.g. Right driver publishing Left_joint*)."""
        is_right = "right" in self._namespace.lower()
        for name in self._joint_names:
            if is_right and name.startswith("Left_"):
                self.get_logger().error(
                    f"joint_names look like the LEFT arm ({name}); "
                    "check right_arm_hardware.yaml is loaded for right_arm_driver"
                )
                return
            if not is_right and name.startswith("Right_"):
                self.get_logger().error(
                    f"joint_names look like the RIGHT arm ({name}); "
                    "check left_arm_hardware.yaml is loaded for left_arm_driver"
                )
                return

    def _float_list_param(
        self, name: str, default_scalar: float, size: int
    ) -> List[float]:
        val = self.get_parameter(name).value
        if isinstance(val, (list, tuple)):
            out = [float(x) for x in val]
            if len(out) < size:
                out.extend([default_scalar] * (size - len(out)))
            return out[:size]
        scalar = float(val)
        return [scalar] * size

    def _load_teleop_config(self) -> None:
        """Read teleop params in configure (launch YAML may not apply at __init__)."""
        self._teleop_backend = str(self.get_parameter("teleop_backend").value).lower()
        self._cartesian_max_lin = float(
            self.get_parameter("cartesian_max_linear_vel").value
        )
        self._cartesian_max_ang = float(
            self.get_parameter("cartesian_max_angular_vel").value
        )

    def _load_joint_control_config(self) -> None:
        """Read the joint control mode / stiffness params in configure."""
        mode = str(self.get_parameter("joint_control_mode").value).lower()
        if mode not in ("position", "impedance"):
            self.get_logger().warn(
                f"joint_control_mode={mode!r} is not 'position' or 'impedance'; "
                "falling back to 'position'"
            )
            mode = "position"
        self._joint_control_mode = mode
        self._joint_stiffness_ratio = self._clamp_stiffness_ratio(
            float(self.get_parameter("joint_stiffness_ratio").value)
        )
        self._goal_settle_timeout = max(
            0.0, float(self.get_parameter("goal_settle_timeout_sec").value)
        )
        self._goal_tol_impedance = float(
            self.get_parameter("goal_joint_tolerance_impedance").value
        )
        self._max_contact_torque = float(
            self.get_parameter("max_contact_torque").value
        )
        if (self._joint_control_mode == "impedance"
                and self._joint_stiffness_ratio >= 0.8):
            self.get_logger().warn(
                f"joint_control_mode is 'impedance' but joint_stiffness_ratio "
                f"is {self._joint_stiffness_ratio}: at nominal K_q the arm is "
                f"as stiff as position mode and will NOT yield to a push — it "
                f"saturates its joint torque instead. Use 0.1-0.3 for "
                f"compliance, and never test it with a hand or arm in the path."
            )
        if self._joint_control_mode == "impedance" and self._max_contact_torque <= 0.0:
            self.get_logger().warn(
                "max_contact_torque is unset, so nothing bounds the torque the "
                "impedance law will demand when the arm is obstructed."
            )
        send_rate = float(self.get_parameter("trajectory_send_rate_hz").value)
        self._traj_send_rate = send_rate if send_rate > 0.0 else self._rate_hz

    def _clamp_stiffness_ratio(self, ratio: float) -> float:
        """K_q is valid in [0, K_q_nom], so the ratio is valid in [0, 1]."""
        if ratio < 0.0 or ratio > 1.0:
            self.get_logger().warn(
                f"joint_stiffness_ratio {ratio} outside [0, 1]; clamping"
            )
        return max(0.0, min(1.0, ratio))

    def _on_set_parameters(self, params):
        """Allow the motion-tuning parameters to be retuned live.

        Retuning is why these are parameters rather than services: finding a
        usable stiffness or velocity cap is an interactive process, and
        `ros2 param set` needs no new message definition.

        default_max_joint_vel / default_max_joint_acc are the caps handed to
        SendJointPosition on every send. They matter for smoothness: each send
        triggers an online re-plan inside the controller (see RDK's warning on
        SendJointPosition), so a cap far above what the commanded trajectory
        actually needs makes every re-plan sprint at the cap and then get
        aborted by the next send. Lowering them toward the trajectory's own
        profile is the first thing to try when motion looks rough.
        """
        for param in params:
            if param.name == "trajectory_send_rate_hz":
                try:
                    value = float(param.value)
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False,
                        reason="trajectory_send_rate_hz must be a float")
                if not 0.0 <= value <= 200.0:
                    return SetParametersResult(
                        successful=False,
                        reason="trajectory_send_rate_hz must be 0..200 "
                               "(0 means use publish_rate_hz)")
                self._traj_send_rate = value if value > 0.0 else self._rate_hz
                self.get_logger().info(
                    f"trajectory send rate set to {self._traj_send_rate:.1f} Hz "
                    f"(applies to the next trajectory)")
                continue
            if param.name in ("default_max_joint_vel", "default_max_joint_acc"):
                try:
                    value = float(param.value)
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be a float")
                if value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be positive")
                size = self._rdk_dof or self._dof
                if param.name == "default_max_joint_vel":
                    self._max_vel = [value] * size
                else:
                    self._max_acc = [value] * size
                self.get_logger().info(
                    f"{param.name} set to {value} on {size} axes "
                    f"(applies to the next SendJointPosition)")
                continue
            if param.name != "joint_stiffness_ratio":
                continue
            try:
                ratio = float(param.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason="joint_stiffness_ratio must be a float",
                )
            if ratio < 0.0 or ratio > 1.0:
                return SetParametersResult(
                    successful=False,
                    reason="joint_stiffness_ratio must be in [0, 1] "
                           "(K_q is valid in [0, K_q_nom])",
                )
            self._joint_stiffness_ratio = ratio
            if self._joint_control_mode != "impedance":
                # Accepted but inert: nothing reads this in position mode, and
                # silently storing it would look like it had taken effect.
                self.get_logger().warn(
                    f"joint_stiffness_ratio set to {ratio}, but "
                    "joint_control_mode is 'position' so it has no effect. Set "
                    "joint_control_mode:=impedance in the driver YAML."
                )
                continue
            # Applies now if already in NRT_JOINT_IMPEDANCE, otherwise the next
            # mode entry (trajectory start or teleop enable) picks it up.
            self._apply_joint_stiffness(quiet=True)
        return SetParametersResult(successful=True)

    def _active_goal_tolerance(self) -> float:
        """Convergence tolerance for the mode actually in use.

        Impedance tracking carries a steady-state position error proportional
        to the unmodelled load, so holding it to the position-mode tolerance is
        asking the arm for something compliance rules out.
        """
        if self._joint_control_mode == "impedance":
            return self._goal_tol_impedance
        return self._goal_tol

    def _joint_rdk_mode(self):
        """The RDK joint mode this driver commands in, per joint_control_mode."""
        rdk = self._session.rdk
        if self._joint_control_mode == "impedance":
            return rdk.Mode.NRT_JOINT_IMPEDANCE
        return rdk.Mode.NRT_JOINT_POSITION

    def _joint_rdk_mode_name(self) -> str:
        return ("NRT_JOINT_IMPEDANCE" if self._joint_control_mode == "impedance"
                else "NRT_JOINT_POSITION")

    def _apply_joint_stiffness(self, quiet: bool = False) -> None:
        """Push joint_stiffness_ratio * K_q_nom, scaling only the arm axes.

        No-op unless we are in impedance mode and actually in the RDK's
        NRT_JOINT_IMPEDANCE mode -- SetJointImpedance raises otherwise.
        """
        if self._mock or not self._session:
            return
        if self._joint_control_mode != "impedance":
            return
        try:
            if mode_to_str(self._session.robot.mode()) != "NRT_JOINT_IMPEDANCE":
                return
            nominal = self._session.joint_stiffness_nominal()
            # Waist entries are +inf nominal and must pass through untouched;
            # only the trailing arm axes are scaled.
            start = max(0, len(nominal) - self._arm_dof)
            K_q = list(nominal)
            for i in range(start, len(nominal)):
                K_q[i] = nominal[i] * self._joint_stiffness_ratio
            self._session.set_joint_impedance(K_q)
            if self._max_contact_torque > 0.0:
                # Same applicable modes as SetJointImpedance, so it goes here:
                # after the mode switch, never before.
                self._session.robot.SetMaxContactTorque(
                    [self._max_contact_torque] * len(nominal)
                )
            if not quiet:
                torque_note = (
                    f", max contact torque {self._max_contact_torque:.0f} Nm"
                    if self._max_contact_torque > 0.0 else ", torque UNBOUNDED"
                )
                self.get_logger().info(
                    f"joint stiffness set to {self._joint_stiffness_ratio} x "
                    f"K_q_nom on axes {start}..{len(nominal) - 1}{torque_note}"
                )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"SetJointImpedance failed: {exc}")

    def _load_timestamp_config(self) -> None:
        """Read stamping params in configure (launch YAML may not apply at __init__)."""
        self._use_device_timestamp = bool(
            self.get_parameter("use_device_timestamp").value
        )
        self._offset_refresh_sec = float(self.get_parameter("offset_refresh_sec").value)
        self._offset_slew_limit = float(self.get_parameter("offset_slew_limit").value)
        self._stale_threshold_sec = float(
            self.get_parameter("stale_threshold_sec").value
        )
        self._publish_effort = bool(self.get_parameter("publish_effort").value)

    def _load_gripper_config(self) -> None:
        self._gripper_enabled = bool(self.get_parameter("gripper_enabled").value)
        self._gripper_name = str(self.get_parameter("gripper_name").value)
        self._gripper_ctrl = ForceGraspController(
            ForceGraspParams(
                open_threshold=float(
                    self.get_parameter("gripper_open_threshold").value
                ),
                grasp_threshold=float(
                    self.get_parameter("gripper_grasp_threshold").value
                ),
                max_force_fraction=float(
                    self.get_parameter("gripper_max_force_fraction").value
                ),
                min_resend_force_delta=float(
                    self.get_parameter("gripper_force_resend_delta").value
                ),
            )
        )

    def _init_gripper_hw(self) -> None:
        if self._mock or not self._session or not self._gripper_enabled:
            return
        if self._gripper is not None and self._gripper.ready:
            return
        try:
            rdk = import_flexivrdk(self.get_logger())
            self._gripper = FlexivGripper(
                self._session.robot,
                rdk,
                self._gripper_name,
                self.get_logger(),
            )
            self._gripper.init()
            if self._gripper_ctrl is not None:
                self._gripper_ctrl.reset()
        except Exception as exc:  # noqa: BLE001
            self._gripper = None
            self.get_logger().error(f"gripper init failed: {exc}")

    def _gripper_command_cb(self, msg: Float32) -> None:
        trigger = float(msg.data)
        if not self._gripper_enabled:
            return
        if self._mock:
            self.get_logger().info(
                f"gripper (mock) trigger={trigger:.2f}", throttle_duration_sec=1.0
            )
            return
        if not self._session or self._gripper_ctrl is None:
            return
        if self._gripper is None or not self._gripper.ready:
            self._init_gripper_hw()
        if self._gripper is None or not self._gripper.ready:
            return
        cmd = self._gripper_ctrl.update(trigger, self._gripper.max_force)
        if cmd is None:
            return
        kind, value = cmd
        try:
            if kind == "open":
                self._gripper.open()
            else:
                self._gripper.grasp(value)
            if self._gripper_width_pub is not None:
                w = Float32()
                w.data = float(self._gripper.width)
                self._gripper_width_pub.publish(w)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"gripper {kind} failed: {exc}", throttle_duration_sec=1.0
            )

    def _note_rdk_send(self, kind: str) -> None:
        """Log measured RDK command rate once per interval (per kind label)."""
        if not self._log_rdk_send_rate:
            return
        now = time.monotonic()
        st = self._rdk_send_stats.get(kind)
        if st is None:
            st = {"count": 0, "t0": now}
            self._rdk_send_stats[kind] = st
        st["count"] += 1
        dt = now - st["t0"]
        if dt < self._log_rdk_send_interval:
            return
        hz = st["count"] / dt
        self.get_logger().info(
            f"RDK {kind}: {hz:.1f} Hz ({st['count']} sends in {dt:.2f}s)"
        )
        st["count"] = 0
        st["t0"] = now

    def _rdk_arm_start_index(self) -> int:
        """Start index of arm joints inside RDK states().q (external axes often come first)."""
        if self._rdk_dof >= self._ext_dof + self._arm_dof and self._ext_dof > 0:
            return self._ext_dof
        if self._rdk_dof == self._arm_dof:
            return 0
        return max(0, self._rdk_dof - self._arm_dof)

    def _extract_arm_q(self, q_full: List[float]) -> List[float]:
        """Map RDK q vector to URDF arm joint order."""
        start = self._rdk_arm_start_index()
        n = min(self._dof, self._arm_dof, max(0, len(q_full) - start))
        return [float(q_full[start + i]) for i in range(n)]

    def _extract_arm_dq(self, dq_full: List[float]) -> List[float]:
        start = self._rdk_arm_start_index()
        n = min(self._dof, self._arm_dof, max(0, len(dq_full) - start))
        return [float(dq_full[start + i]) for i in range(n)]

    def _init_rdk_dof_from_robot(self) -> None:
        """RDK session may include external (waist) axes in SendJointPosition."""
        info = self._session.robot.info()
        self._rdk_dof = int(info.DoF)
        self._arm_dof = int(info.DoF_m)
        self._ext_dof = int(info.DoF_e)
        self.get_logger().info(
            f"RDK DoF={self._rdk_dof} (arm DoF_m={self._arm_dof}, "
            f"external DoF_e={self._ext_dof}); trajectory commands use "
            f"{self._dof} URDF joints, external axes held at current q"
        )
        if self._arm_dof != self._dof:
            self.get_logger().warn(
                f"URDF joint_names count ({self._dof}) != RDK DoF_m ({self._arm_dof})"
            )
        self._max_vel = self._float_list_param(
            "default_max_joint_vel", 1.5, self._rdk_dof
        )
        self._max_acc = self._float_list_param(
            "default_max_joint_acc", 3.0, self._rdk_dof
        )

    def _expand_arm_to_rdk(
        self, q_arm: List[float], dq_arm: List[float]
    ) -> tuple[List[float], List[float]]:
        """Pad 7-DOF arm trajectory with current external-axis positions."""
        st = self._session.states()
        q_full = [float(x) for x in st.q[: self._rdk_dof]]
        dq_full = [float(x) for x in st.dq[: self._rdk_dof]]
        start = self._rdk_arm_start_index()
        n = min(len(q_arm), self._arm_dof, len(q_full) - start)
        for i in range(n):
            q_full[start + i] = q_arm[i]
            dq_full[start + i] = dq_arm[i] if i < len(dq_arm) else 0.0
        return q_full, dq_full

    def _reset_servo_stream_state(self) -> None:
        self._servo_prev_q = None
        self._servo_prev_time = None

    def _compute_servo_dq(
        self, q_cmd: List[float], dq_msg: List[float]
    ) -> List[float]:
        """Terminal velocity for RDK SendJointPosition during Servo streaming.

        Flexiv treats dq as velocity *at the target*. dq≈0 means stop at each
        waypoint (jerky / barely moves). Plan & Execute interpolates dq; Servo
        often omits velocities — finite-difference from successive setpoints.
        """
        now = time.monotonic()
        dq = (
            list(dq_msg[: self._dof])
            if len(dq_msg) >= self._dof
            else [0.0] * self._dof
        )
        while len(dq) < self._dof:
            dq.append(0.0)

        if (
            all(abs(v) < 1e-6 for v in dq)
            and self._servo_prev_q is not None
            and self._servo_prev_time is not None
        ):
            dt = now - self._servo_prev_time
            if dt > 1e-4:
                dq = [
                    (q_cmd[i] - self._servo_prev_q[i]) / dt
                    for i in range(self._dof)
                ]

        start = self._rdk_arm_start_index()
        for i in range(self._dof):
            cap_idx = start + i
            cap = (
                float(self._max_vel[cap_idx])
                if cap_idx < len(self._max_vel)
                else 1.5
            )
            dq[i] = max(-cap, min(cap, dq[i]))

        self._servo_prev_q = list(q_cmd)
        self._servo_prev_time = now
        return dq

    def _enter_servo_rdk_mode(self) -> str:
        """Stop + the configured joint mode; unlock waist axes on the left arm."""
        if self._mock or not self._session:
            return "MOCK"
        self._session.stop()
        time.sleep(0.05)
        # LockExternalAxes requires IDLE; must run before SwitchMode (same as rdk_joint_jog_gui).
        if self._ext_dof > 0 and "left" in self._namespace.lower():
            self._session.robot.LockExternalAxes(False)
        self._session.switch_mode(self._joint_rdk_mode())
        # Stiffness has to be set after the mode switch, not before.
        self._apply_joint_stiffness()
        return mode_to_str(self._session.robot.mode())

    def _reset_cartesian_stream_state(self) -> None:
        self._cartesian_pose = None
        self._cartesian_prev_t = None

    def _seed_cartesian_from_tcp(self) -> None:
        if self._mock or not self._session:
            self._cartesian_pose = None
            return
        tcp = list(self._session.states().tcp_pose)
        if len(tcp) < 7:
            self._cartesian_pose = None
            return
        self._cartesian_pose = [float(x) for x in tcp[:7]]
        self._cartesian_prev_t = time.monotonic()

    def _send_cartesian_pose(self, pose: List[float]) -> None:
        if self._mock or not self._session:
            return
        try:
            mode = mode_to_str(self._session.robot.mode())
            if mode != "NRT_CARTESIAN_MOTION_FORCE":
                mode = self._enter_cartesian_rdk_mode()
                if mode != "NRT_CARTESIAN_MOTION_FORCE":
                    return
            self._session.send_cartesian_motion_force(
                pose,
                max_linear_vel=self._cartesian_max_lin,
                max_angular_vel=self._cartesian_max_ang,
            )
            self._note_rdk_send("SendCartesianMotionForce")
        except Exception as exc:  # noqa: BLE001
            fault, operational, _, mode, detail = self._session.status_snapshot()
            self.get_logger().error(
                f"cartesian SendCartesianMotionForce failed: {exc}; mode={mode}, "
                f"operational={operational}, fault={fault}, detail={detail}",
                throttle_duration_sec=1.0,
            )

    def _hold_cartesian_at_tcp(self) -> None:
        """Re-sync command pose to measured TCP and stream a hold setpoint."""
        self._seed_cartesian_from_tcp()
        if self._cartesian_pose is None:
            return
        self._send_cartesian_pose(self._cartesian_pose)

    def _enter_cartesian_rdk_mode(self) -> str:
        """Stop + NRT_CARTESIAN_MOTION_FORCE; unlock waist on the left arm."""
        if self._mock or not self._session:
            return "MOCK"
        mode = mode_to_str(self._session.robot.mode())
        if mode == "NRT_CARTESIAN_MOTION_FORCE":
            return mode
        self._session.stop()
        time.sleep(0.05)
        if self._ext_dof > 0 and "left" in self._namespace.lower():
            self._session.robot.LockExternalAxes(False)
        self._session.switch_mode(
            self._session.rdk.Mode.NRT_CARTESIAN_MOTION_FORCE
        )
        return mode_to_str(self._session.robot.mode())

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._load_teleop_config()
        self._load_joint_control_config()
        self._load_timestamp_config()
        self._load_gripper_config()
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.get_logger().info(
            f"configure: mock={self._mock} sn={self._robot_sn or '(unset)'} "
            f"joints={self._joint_names} teleop_backend={self._teleop_backend} "
            f"joint_control_mode={self._joint_control_mode} "
            f"stiffness_ratio={self._joint_stiffness_ratio} "
            f"goal_tol={math.degrees(self._active_goal_tolerance()):.2f}deg "
            f"traj_send={self._traj_send_rate:.0f}Hz "
            f"gripper={'on' if self._gripper_enabled else 'off'}"
        )
        self._hw_ready = False
        if not self._mock:
            if not self._robot_sn:
                self.get_logger().error(
                    "mock_hardware:=false requires robot_sn (Flexiv Elements serial)"
                )
                return TransitionCallbackReturn.FAILURE
            try:
                self._session = FlexivSession(self._robot_sn, self.get_logger())
                self._session.connect()
                self._init_rdk_dof_from_robot()
                self._session.clear_fault()
                self._hw_ready = True
                # Per-controller: the two arms' clocks differ (sec 4.4), so
                # each driver owns its own estimate — never share one.
                self._offset = OffsetTracker(
                    states_fn=self._session.states,
                    refresh_sec=self._offset_refresh_sec,
                    slew_limit_sec=self._offset_slew_limit,
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"flexivrdk connect failed: {exc}")
                self._session = None
                return TransitionCallbackReturn.FAILURE

        self._js_pub = self.create_lifecycle_publisher(
            JointState, "joint_states", qos_profile_sensor_data
        )
        self._status_pub = self.create_lifecycle_publisher(ArmStatus, "status", 10)
        self._tcp_pub = self.create_lifecycle_publisher(PoseStamped, "tcp_pose", 10)
        self._wrench_pub = self.create_lifecycle_publisher(
            WrenchStamped, "external_wrench", 10
        )
        self._fault_pub = self.create_lifecycle_publisher(Bool, "fault", 10)
        self._mode_pub = self.create_lifecycle_publisher(String, "mode", 10)
        if self._gripper_enabled:
            self._gripper_width_pub = self.create_lifecycle_publisher(
                Float32, "gripper/width", 10
            )
            self._gripper_sub = self.create_subscription(
                Float32,
                "gripper_command",
                self._gripper_command_cb,
                10,
                callback_group=self._cb_group,
            )
        self._exec_prim_srv = self.create_service(
            ExecutePrimitive, "execute_primitive", self._handle_execute_primitive
        )
        self._clear_fault_srv = self.create_service(
            Trigger, "clear_fault", self._handle_clear_fault
        )
        self._traj_server = ActionServer(
            self,
            FollowJointTrajectory,
            "follow_joint_trajectory",
            execute_callback=self._execute_trajectory,
            goal_callback=self._traj_goal_callback,
            cancel_callback=self._traj_cancel_callback,
            callback_group=self._cb_group,
        )
        self._set_teleop_srv = self.create_service(
            SetBool,
            "set_teleop_mode",
            self._handle_set_teleop_mode,
            callback_group=self._cb_group,
        )
        if self._teleop_backend == "cartesian":
            self._cartesian_sub = self.create_subscription(
                TwistStamped,
                "cartesian_twist_cmds",
                self._cartesian_twist_cb,
                10,
                callback_group=self._cb_group,
            )
        else:
            self._servo_sub = self.create_subscription(
                JointTrajectory,
                "servo_joint_command",
                self._servo_command_cb,
                10,
                callback_group=self._cb_group,
            )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        for pub in (
            self._js_pub,
            self._status_pub,
            self._tcp_pub,
            self._wrench_pub,
            self._fault_pub,
            self._mode_pub,
            self._gripper_width_pub,
        ):
            if pub is not None:
                pub.on_activate(state)

        if not self._mock and self._session and self._auto_enable:
            if not self._session.enable(self._op_timeout):
                self.get_logger().warn(
                    "Arm not operational — check fault, e-stop, motion bar Auto Remote"
                )
        if self._gripper_enabled:
            self._init_gripper_hw()

        if not self._mock and self._session:
            if self._use_device_timestamp and self._offset is not None:
                # Estimate before the poll thread starts, while nothing else
                # is contending for states() calls.
                try:
                    est = self._offset.refresh()
                    self.get_logger().info(
                        f"clock offset: {est.offset_sec:.6f}s "
                        f"(spread {est.spread_sec * 1e6:.1f}us, "
                        f"{est.n_transitions} transitions)"
                    )
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(
                        f"initial offset estimate failed, falling back to now(): {exc}"
                    )
            self._poll_stop.clear()
            self._poll_thread = threading.Thread(
                target=self._poll_loop, name=f"{self._namespace}_poll", daemon=True
            )
            self._poll_thread.start()

        self._t0 = self.get_clock().now().nanoseconds * 1e-9
        if self._timer is None:
            self._timer = self.create_timer(1.0 / self._rate_hz, self._publish_state)
        self.get_logger().info(
            f"activate: /{self._namespace}/* live "
            f"(publish {self._rate_hz} Hz, device_timestamp="
            f"{self._use_device_timestamp})"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._teleop_active = False
        self._cancel_active_trajectory()
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        self._poll_stop.set()
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=2.0)
            self._poll_thread = None
        if not self._mock and self._session:
            self._session.stop()
        for pub in (
            self._js_pub,
            self._status_pub,
            self._tcp_pub,
            self._wrench_pub,
            self._fault_pub,
            self._mode_pub,
            self._gripper_width_pub,
        ):
            if pub is not None:
                pub.on_deactivate(state)
        self.get_logger().info("deactivate")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._cancel_active_trajectory()
        if self._traj_server is not None:
            self._traj_server.destroy()
            self._traj_server = None
        if self._servo_sub is not None:
            self.destroy_subscription(self._servo_sub)
            self._servo_sub = None
        if self._cartesian_sub is not None:
            self.destroy_subscription(self._cartesian_sub)
            self._cartesian_sub = None
        if self._gripper_sub is not None:
            self.destroy_subscription(self._gripper_sub)
            self._gripper_sub = None
        if self._set_teleop_srv is not None:
            self.destroy_service(self._set_teleop_srv)
            self._set_teleop_srv = None
        if self._exec_prim_srv is not None:
            self.destroy_service(self._exec_prim_srv)
            self._exec_prim_srv = None
        if self._clear_fault_srv is not None:
            self.destroy_service(self._clear_fault_srv)
            self._clear_fault_srv = None
        for pub in (
            self._js_pub,
            self._status_pub,
            self._tcp_pub,
            self._wrench_pub,
            self._fault_pub,
            self._mode_pub,
            self._gripper_width_pub,
        ):
            if pub is not None:
                self.destroy_lifecycle_publisher(pub)
        self._js_pub = None
        self._status_pub = None
        self._tcp_pub = None
        self._wrench_pub = None
        self._fault_pub = None
        self._mode_pub = None
        self._gripper_width_pub = None
        self._gripper = None
        if self._gripper_ctrl is not None:
            self._gripper_ctrl.reset()
        if self._session is not None:
            self._session.disconnect()
            self._session = None
        self._offset = None
        self._sample = None
        self._hw_ready = False
        self.get_logger().info("cleanup")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _poll_loop(self) -> None:
        """Free-running acquisition: notice every device tick, store the latest.

        Acquisition rate is set by the device (1 kHz, verified on hardware),
        not by publish_rate_hz — which is now an honest decimation factor
        rather than an aperiodic sampler of a 1 kHz signal (sec 2a/4.1).
        """
        last_ts: Optional[Tuple[int, int]] = None
        # Instrumented because the 1 kHz claim above is not what /joint_states
        # shows: during a trajectory a third of consecutive published samples
        # repeat, which means this slot is not being refreshed between
        # publishes. Two candidate causes, and these counters tell them apart:
        # polls/s far below 1 kHz means this thread is starved (it spins with
        # no sleep, so it is CPU-bound and loses GIL slices to every thread
        # doing blocking work), while polls/s high with updates/s low means the
        # device stream itself is slower than expected.
        polls = 0
        updates = 0
        window_t0 = time.monotonic()
        while not self._poll_stop.is_set():
            try:
                st = self._session.states()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"states() failed: {exc}", throttle_duration_sec=1.0
                )
                time.sleep(0.05)
                continue
            # (sec, nanosec) tuple — compare exactly, don't cast to float here.
            polls += 1
            ts = st.timestamp
            if ts != last_ts:
                last_ts = ts
                updates += 1
                self._sample = _ArmSample(
                    device_timestamp=ts,
                    host_mono=time.monotonic(),
                    q=[float(x) for x in st.q[: self._rdk_dof]],
                    dq=[float(x) for x in st.dq[: self._rdk_dof]],
                    tau=[float(x) for x in st.tau[: self._rdk_dof]],
                    tcp_pose=[float(x) for x in st.tcp_pose],
                    ext_wrench=[float(x) for x in st.ext_wrench_in_tcp],
                )
            # No sleep: states() is a cached read (~4 us, sec 1).
            if self._log_rdk_send_rate:
                elapsed = time.monotonic() - window_t0
                if elapsed >= self._log_rdk_send_interval:
                    self.get_logger().info(
                        f"acquisition: {polls / elapsed:.0f} polls/s, "
                        f"{updates / elapsed:.0f} new samples/s "
                        f"(design assumes ~1000; a publish at "
                        f"{self._rate_hz:.0f} Hz repeats a sample whenever "
                        f"this drops below it)"
                    )
                    polls = 0
                    updates = 0
                    window_t0 = time.monotonic()

    def _maybe_refresh_offset(self) -> None:
        if (
            not self._use_device_timestamp
            or self._offset is None
            or not self._offset.due()
        ):
            return
        try:
            self._offset.refresh()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"offset refresh failed: {exc}", throttle_duration_sec=5.0
            )

    def _extract_arm_tau(self, tau_full: List[float]) -> List[float]:
        start = self._rdk_arm_start_index()
        n = min(self._dof, self._arm_dof, max(0, len(tau_full) - start))
        return [float(tau_full[start + i]) for i in range(n)]

    def _publish_state(self) -> None:
        if self._js_pub is None:
            return

        stamp = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = stamp
        js.name = self._joint_names

        if self._mock:
            # _traj_t0 is ROS time elapsed since activate (matches _execute_trajectory).
            t_since_activate = (
                self.get_clock().now().nanoseconds * 1e-9 - self._t0
            )
            if self._active_traj is not None:
                elapsed = t_since_activate - self._traj_t0
                q, dq = sample_trajectory(self._active_traj, elapsed, self._dof)
            else:
                q = list(self._mock_hold_q)
                dq = [0.0] * self._dof
            js.position = q
            js.velocity = dq
            fault, operational, enabled = False, True, True
            mode, detail = "MOCK", "mock_hardware"
            tcp_pose = PoseStamped()
            tcp_pose.header.stamp = stamp
            tcp_pose.header.frame_id = self._tcp_frame
            tcp_pose.pose.orientation.w = 1.0
            wrench = WrenchStamped()
            wrench.header.stamp = stamp
            wrench.header.frame_id = self._tcp_frame
        else:
            if not self._session or not self._session.robot:
                return
            self._maybe_refresh_offset()

            sample = self._sample
            if sample is None:
                self.get_logger().warn(
                    "no arm sample yet",
                    throttle_duration_sec=_STALE_WARN_INTERVAL_SEC,
                )
                return
            age = time.monotonic() - sample.host_mono
            if age > self._stale_threshold_sec:
                self.get_logger().warn(
                    f"arm sample stale ({age * 1e3:.1f} ms); skipping publish",
                    throttle_duration_sec=_STALE_WARN_INTERVAL_SEC,
                )
                return

            if self._use_device_timestamp and self._offset is not None:
                stamp = seconds_to_ros_time(
                    self._offset.to_ros_seconds(sample.device_timestamp)
                )
                js.header.stamp = stamp

            # q[0:2] are the waist external axes and belong to
            # aico2_waist_driver — _extract_arm_* starts at _rdk_arm_start_index
            # so this driver publishes only its own 7 joints. Two publishers
            # claiming one joint makes TF jitter between them (sec 4.3).
            js.position = self._extract_arm_q(sample.q)
            js.velocity = self._extract_arm_dq(sample.dq)
            if self._publish_effort:
                js.effort = self._extract_arm_tau(sample.tau)
            fault, operational, enabled, mode, detail = self._session.status_snapshot()
            tcp_pose = PoseStamped()
            tcp_pose.header.stamp = stamp
            tcp_pose.header.frame_id = self._tcp_frame
            tcp_pose.pose = tcp_pose_to_ros(sample.tcp_pose)
            wrench = WrenchStamped()
            wrench.header.stamp = stamp
            wrench.header.frame_id = self._tcp_frame
            wrench.wrench = wrench6_to_ros(sample.ext_wrench)

        self._js_pub.publish(js)
        status = ArmStatus()
        status.fault = fault
        status.operational = operational
        status.enabled = enabled
        status.mode = mode
        status.detail = detail
        self._status_pub.publish(status)
        self._fault_pub.publish(Bool(data=fault))
        self._mode_pub.publish(String(data=mode))
        self._tcp_pub.publish(tcp_pose)
        self._wrench_pub.publish(wrench)

    def _traj_goal_callback(self, goal_request):
        if self._teleop_active:
            self.get_logger().warn("Reject trajectory: teleop (servo) mode active")
            return GoalResponse.REJECT
        if not self._mock and (not self._session or not self._hw_ready):
            return GoalResponse.REJECT
        traj = goal_request.trajectory
        if not traj.points:
            self.get_logger().warn("Reject trajectory: no points")
            return GoalResponse.REJECT
        try:
            reorder_to_driver(traj, self._joint_names)
        except ValueError as exc:
            self.get_logger().warn(f"Reject trajectory: {exc}")
            return GoalResponse.REJECT
        if not self._mock and self._session and not self._session.robot.operational():
            self.get_logger().warn("Reject trajectory: arm not operational")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _traj_cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _mock_latch_trajectory_position(self) -> None:
        """Keep mock joint_state at the last commanded point (used after traj ends)."""
        if not self._mock or self._active_traj is None:
            return
        t_since_activate = self.get_clock().now().nanoseconds * 1e-9 - self._t0
        elapsed = t_since_activate - self._traj_t0
        q, _ = sample_trajectory(self._active_traj, elapsed, self._dof)
        self._mock_hold_q = list(q[: self._dof])

    def _cancel_active_trajectory(self) -> None:
        if self._mock:
            self._mock_latch_trajectory_position()
        if self._traj_goal_handle is not None:
            try:
                self._traj_goal_handle.canceled()
            except Exception:  # noqa: BLE001
                pass
            self._traj_goal_handle = None
        self._active_traj = None
        if not self._mock and self._session:
            self._session.stop()

    async def _execute_trajectory(self, goal_handle):
        traj = goal_handle.request.trajectory
        try:
            _, traj = reorder_to_driver(traj, self._joint_names)
        except ValueError as exc:
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_string = str(exc)
            return result

        # Set _traj_t0 before _active_traj so the 50 Hz publisher never samples with
        # a new trajectory but a stale _traj_t0 (that produced huge elapsed / jumps).
        self._traj_t0 = self.get_clock().now().nanoseconds * 1e-9 - self._t0
        self._active_traj = traj
        self._traj_goal_handle = goal_handle
        duration = trajectory_duration(traj)
        if not self._mock and self._session:
            # Ensure previous sessions/commands are stopped before switching mode.
            self._session.stop()
            time.sleep(0.05)
            try:
                self._session.switch_mode(self._joint_rdk_mode())
                # After the switch, never before: SetJointImpedance is only
                # applicable in NRT_JOINT_IMPEDANCE.
                self._apply_joint_stiffness()
            except Exception as exc:  # noqa: BLE001
                fault, operational, _, mode, detail = self._session.status_snapshot()
                detail_suffix = f", detail={detail}" if detail else ""
                msg = (
                    f"SwitchMode({self._joint_rdk_mode_name()}) failed: "
                    f"{exc}; fault={fault}, operational={operational}, mode={mode}"
                    f"{detail_suffix}"
                )
                self.get_logger().error(msg)
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                # Must be set explicitly: the field defaults to 0, which is
                # SUCCESSFUL, so an aborted goal would otherwise report success
                # to any client that reads error_code without also checking the
                # goal status.
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = msg
                self._active_traj = None
                self._traj_goal_handle = None
                return result

        feedback = FollowJointTrajectory.Feedback()
        rate = 1.0 / self._traj_send_rate
        start = time.monotonic()
        # Counted so the achieved rate can be compared against the configured
        # one: they diverged badly and nothing reported it.
        iterations = 0

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    # MoveIt cancels when execution outruns
                    # allowed_execution_duration_scaling, so say so: otherwise
                    # this looks like a user cancel in the log.
                    self.get_logger().warn(
                        f"trajectory cancelled by the client "
                        f"{time.monotonic() - start:.1f}s in (duration "
                        f"{duration:.1f}s)"
                    )
                    self._cancel_active_trajectory()
                    goal_handle.canceled()
                    result = FollowJointTrajectory.Result()
                    result.error_code = (
                        FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    )
                    result.error_string = "cancelled by the client"
                    return result

                elapsed = time.monotonic() - start
                iterations += 1
                q_cmd, dq_cmd = sample_trajectory(traj, elapsed, self._dof)

                if not self._mock and self._session:
                    try:
                        q_send, dq_send = self._expand_arm_to_rdk(q_cmd, dq_cmd)
                        self._session.send_joint_position(
                            q_send, dq_send, self._max_vel, self._max_acc
                        )
                        self._note_rdk_send("SendJointPosition/trajectory")
                    except Exception as exc:  # noqa: BLE001
                        msg = f"SendJointPosition failed: {exc}"
                        self.get_logger().error(msg)
                        goal_handle.abort()
                        result = FollowJointTrajectory.Result()
                        # Mid-path hardware failure. Not literally a tolerance
                        # violation, but it is the action's code for "the
                        # trajectory stopped following the path", and any
                        # non-zero value beats reporting SUCCESSFUL.
                        result.error_code = (
                            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                        )
                        result.error_string = msg
                        self._active_traj = None
                        self._traj_goal_handle = None
                        return result

                # Read the poll thread's slot rather than making a second
                # blocking RDK call: feedback is informational, and the extra
                # round trip per iteration is what caps this loop near 25 Hz
                # however high trajectory_send_rate_hz is set. The convergence
                # check below still takes a fresh reading, because that one
                # decides when the goal succeeds.
                sample = self._sample
                if self._mock or not self._session:
                    feedback.actual.positions = q_cmd
                elif sample is not None:
                    feedback.actual.positions = self._extract_arm_q(
                        list(sample.q[: self._rdk_dof])
                    )
                else:
                    feedback.actual.positions = q_cmd
                feedback.desired.positions = q_cmd
                feedback.desired.time_from_start.sec = int(elapsed)
                feedback.desired.time_from_start.nanosec = int(
                    (elapsed % 1.0) * 1e9
                )
                goal_handle.publish_feedback(feedback)

                at_goal = elapsed >= duration
                err = 0.0
                worst_joint = ""
                if at_goal and not self._mock and self._session:
                    st = self._session.states()
                    q_act = self._extract_arm_q(
                        [float(x) for x in st.q[: self._rdk_dof]]
                    )
                    q_goal = q_cmd[: self._arm_dof]
                    deltas = [abs(a - b) for a, b in zip(q_act, q_goal)]
                    err = max(deltas)
                    idx = deltas.index(err)
                    worst_joint = (self._joint_names[idx]
                                   if idx < len(self._joint_names) else f"[{idx}]")
                    goal_tol = self._active_goal_tolerance()
                    at_goal = err < goal_tol
                    # Bounded wait, and deliberately shorter than the client's
                    # own limit so the driver is what explains the failure.
                    # MoveIt cancels at duration * allowed_execution_duration_
                    # scaling (2.0) + allowed_goal_duration_margin (1.0); the
                    # 0.5*duration + 0.5 cap stays below that for any duration,
                    # which a fixed allowance does not (at duration 2.1s a 3.0s
                    # allowance lands 0.1s before MoveIt, which is no margin).
                    settle = min(self._goal_settle_timeout,
                                 0.5 * duration + 0.5)
                    if not at_goal and elapsed - duration > settle:
                        msg = (
                            f"goal tolerance not reached: {worst_joint} is "
                            f"{math.degrees(err):.2f}° from target "
                            f"({math.degrees(goal_tol):.2f}° allowed in "
                            f"{self._joint_control_mode} mode) "
                            f"{settle:.1f}s after the "
                            f"trajectory ended. The arm is not tracking the "
                            f"commanded position. In impedance mode a residual "
                            f"this size is usually gravity sag from an "
                            f"undeclared tool — declare it in Flexiv Elements, "
                            f"or raise goal_joint_tolerance_impedance. "
                            f"Otherwise check for a fault."
                        )
                        self.get_logger().error(msg)
                        self._session.stop()
                        goal_handle.abort()
                        result = FollowJointTrajectory.Result()
                        result.error_code = (
                            FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                        )
                        result.error_string = msg
                        self._active_traj = None
                        self._traj_goal_handle = None
                        return result
                elif at_goal and self._mock:
                    at_goal = True

                if at_goal:
                    if elapsed > 0:
                        achieved = iterations / elapsed
                        note = ("" if achieved >= 0.9 * self._traj_send_rate
                                else "  <- loop is saturated; the configured "
                                     "rate is not being reached")
                        self.get_logger().info(
                            f"trajectory sent {iterations} setpoints in "
                            f"{elapsed:.2f}s = {achieved:.0f} Hz "
                            f"(configured {self._traj_send_rate:.0f} Hz){note}"
                        )
                    if self._mock:
                        q_final, _ = sample_trajectory(traj, duration, self._dof)
                        self._mock_hold_q = list(q_final[: self._dof])
                    if not self._mock and self._session:
                        self._session.stop()
                    goal_handle.succeed()
                    result = FollowJointTrajectory.Result()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    self._active_traj = None
                    self._traj_goal_handle = None
                    return result

                time.sleep(rate)
        finally:
            self._active_traj = None
            self._traj_goal_handle = None

        goal_handle.abort()
        return FollowJointTrajectory.Result()

    def _handle_execute_primitive(self, request, response):
        name = request.primitive.strip()
        if not name:
            response.success = False
            response.message = "empty primitive name"
            return response
        if self._mock:
            self.get_logger().info(f"execute_primitive (mock): {name}")
            response.success = True
            response.message = "accepted in mock mode"
            return response
        if not self._session or not self._session.robot.operational():
            response.success = False
            response.message = "arm not operational"
            return response
        try:
            self._session.execute_primitive_name(name)
            response.success = True
            response.message = f"started primitive '{name}' (no params)"
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
        return response

    def _handle_clear_fault(self, request, response):
        if self._mock:
            response.success = True
            response.message = "noop in mock mode"
            return response
        if not self._session:
            response.success = False
            response.message = "not connected"
            return response
        ok = self._session.clear_fault()
        response.success = ok
        response.message = "fault cleared" if ok else "ClearFault failed"
        return response

    def _handle_set_teleop_mode(self, request, response):
        """Enable/disable VR teleop (servo joint stream or Cartesian RDK)."""
        enable = bool(request.data)
        cartesian = self._teleop_backend == "cartesian"
        mode_name = ("NRT_CARTESIAN_MOTION_FORCE" if cartesian
                     else self._joint_rdk_mode_name())
        if enable:
            if self._active_traj is not None:
                response.success = False
                response.message = "cannot enter teleop: trajectory in progress"
                return response
            if not self._mock and self._session:
                try:
                    mode = (
                        self._enter_cartesian_rdk_mode()
                        if cartesian
                        else self._enter_servo_rdk_mode()
                    )
                except Exception as exc:  # noqa: BLE001
                    response.success = False
                    response.message = f"SwitchMode({mode_name}) failed: {exc}"
                    return response
                if mode != mode_name:
                    response.success = False
                    response.message = (
                        f"teleop enable failed: RDK mode is {mode}, "
                        f"expected {mode_name} (check motion bar / fault)"
                    )
                    return response
            self._teleop_active = True
            if cartesian:
                self._hold_cartesian_at_tcp()
            else:
                self._reset_servo_stream_state()
            self.get_logger().info(
                f"teleop ({self._teleop_backend}) mode ENABLED"
            )
            response.success = True
            response.message = "teleop enabled"
        else:
            self._teleop_active = False
            self._reset_servo_stream_state()
            self._reset_cartesian_stream_state()
            if not self._mock and self._session:
                self._session.stop()
            self.get_logger().info(
                f"teleop ({self._teleop_backend}) mode DISABLED"
            )
            response.success = True
            response.message = "teleop disabled"
        return response

    def _cartesian_twist_cb(self, msg: TwistStamped) -> None:
        """Integrate stamped twists and stream SendCartesianMotionForce."""
        if not self._teleop_active or self._active_traj is not None:
            return
        if self._mock or not self._session:
            return
        if self._cartesian_pose is None:
            self._seed_cartesian_from_tcp()
        if self._cartesian_pose is None:
            return

        now = time.monotonic()
        if self._cartesian_prev_t is None:
            self._cartesian_prev_t = now
            return
        dt = min(now - self._cartesian_prev_t, 0.1)
        self._cartesian_prev_t = now
        if dt <= 1e-4:
            return

        lin = [
            float(msg.twist.linear.x),
            float(msg.twist.linear.y),
            float(msg.twist.linear.z),
        ]
        ang = [
            float(msg.twist.angular.x),
            float(msg.twist.angular.y),
            float(msg.twist.angular.z),
        ]
        if all(abs(v) < 1e-6 for v in lin) and all(abs(v) < 1e-6 for v in ang):
            self._hold_cartesian_at_tcp()
            return

        self._cartesian_pose = integrate_tcp_pose(
            self._cartesian_pose, lin, ang, dt
        )
        self._send_cartesian_pose(self._cartesian_pose)

    def _servo_command_cb(self, msg: JointTrajectory) -> None:
        """Stream one Servo setpoint to the arm (NRT joint position).

        Only acts while teleop is active and no trajectory goal is running. Uses
        the last point of the message as the target; in mock mode it latches the
        published joint_states so RViz tracks.
        """
        if not self._teleop_active or self._active_traj is not None:
            return
        if not msg.points:
            return
        try:
            _, traj = reorder_to_driver(msg, self._joint_names)
        except ValueError as exc:
            self.get_logger().warn(
                f"servo cmd rejected: {exc}", throttle_duration_sec=2.0
            )
            return
        point = traj.points[-1]
        q_cmd = [float(x) for x in point.positions[: self._dof]]
        if len(q_cmd) != self._dof:
            return
        dq_from_msg = (
            [float(x) for x in point.velocities[: self._dof]]
            if point.velocities
            else []
        )
        dq_cmd = self._compute_servo_dq(q_cmd, dq_from_msg)
        if self._mock:
            self._mock_hold_q = list(q_cmd)
            return
        if not self._session:
            return
        try:
            q_send, dq_send = self._expand_arm_to_rdk(q_cmd, dq_cmd)
            self._session.send_joint_position(
                q_send, dq_send, self._max_vel, self._max_acc
            )
            self._note_rdk_send("SendJointPosition/servo")
        except Exception as exc:  # noqa: BLE001
            fault, operational, _, mode, detail = self._session.status_snapshot()
            self.get_logger().error(
                f"servo SendJointPosition failed: {exc}; mode={mode}, "
                f"operational={operational}, fault={fault}, detail={detail}",
                throttle_duration_sec=1.0,
            )
            now = time.monotonic()
            if now - self._servo_mode_retry_t > 1.0:
                self._servo_mode_retry_t = now
                try:
                    new_mode = self._enter_servo_rdk_mode()
                    self.get_logger().warn(
                        f"re-entered servo RDK mode -> {new_mode}",
                        throttle_duration_sec=2.0,
                    )
                except Exception as retry_exc:  # noqa: BLE001
                    self.get_logger().error(
                        f"servo mode re-entry failed: {retry_exc}",
                        throttle_duration_sec=2.0,
                    )


def spin_arm_driver(node_name: str, namespace: str) -> None:
    rclpy.init()
    node = ArmDriverNode(node_name, namespace)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
