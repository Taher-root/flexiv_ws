"""Thin wrapper around flexivrdk 1.9 for the left arm lifecycle driver."""

from __future__ import annotations

import time
from typing import List, Optional, Tuple

from geometry_msgs.msg import Pose, Wrench
from rclpy.impl.rcutils_logger import RcutilsLogger


def import_flexivrdk(logger: RcutilsLogger):
    try:
        import flexivrdk  # noqa: WPS433
    except ImportError as exc:
        logger.error(
            "flexivrdk not installed. Use: "
            "/usr/bin/python3.10 -m pip install 'flexivrdk==1.9.0'"
        )
        raise exc
    return flexivrdk


def mode_to_str(mode) -> str:
    return str(mode).split(".")[-1] if mode is not None else "UNKNOWN"


def tcp_pose_to_ros(tcp: List[float]) -> Pose:
    """RDK tcp_pose is [x, y, z, qw, qx, qy, qz] in meters."""
    pose = Pose()
    if len(tcp) < 7:
        return pose
    pose.position.x = float(tcp[0])
    pose.position.y = float(tcp[1])
    pose.position.z = float(tcp[2])
    pose.orientation.w = float(tcp[3])
    pose.orientation.x = float(tcp[4])
    pose.orientation.y = float(tcp[5])
    pose.orientation.z = float(tcp[6])
    return pose


def wrench6_to_ros(wrench: List[float]) -> Wrench:
    w = Wrench()
    if len(wrench) < 6:
        return w
    w.force.x = float(wrench[0])
    w.force.y = float(wrench[1])
    w.force.z = float(wrench[2])
    w.torque.x = float(wrench[3])
    w.torque.y = float(wrench[4])
    w.torque.z = float(wrench[5])
    return w


class FlexivSession:
    """Connect, enable, and command one Rizon arm via flexivrdk."""

    def __init__(self, serial: str, logger: RcutilsLogger) -> None:
        self._serial = serial.strip()
        self._logger = logger
        self._flexivrdk = import_flexivrdk(logger)
        self._robot = None

    @property
    def robot(self):
        return self._robot

    @property
    def rdk(self):
        return self._flexivrdk

    def connect(self) -> None:
        if not self._serial:
            raise ValueError("robot_sn is empty; set serial from Flexiv Elements")
        self._logger.info(f"Connecting flexivrdk Robot({self._serial})")
        self._robot = self._flexivrdk.Robot(self._serial)
        self._logger.info("flexivrdk connected")

    def disconnect(self) -> None:
        self._robot = None

    def clear_fault(self) -> bool:
        if self._robot is None:
            return False
        if not self._robot.fault():
            return True
        self._logger.warn("Arm fault set — calling ClearFault()")
        ok = self._robot.ClearFault()
        if not ok:
            self._logger.error("ClearFault() failed")
        return bool(ok)

    def enable(self, timeout_sec: float) -> bool:
        if self._robot is None:
            return False
        if self._robot.fault() and not self.clear_fault():
            return False
        self._logger.info("Enable() — ensure E-stop released and motion bar Auto Remote")
        self._robot.Enable()
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._robot.operational():
                self._logger.info("Arm operational")
                return True
            time.sleep(0.5)
        self._logger.warn(f"Not operational after {timeout_sec:.0f}s")
        return False

    def stop(self) -> None:
        if self._robot is not None:
            try:
                self._robot.Stop()
            except Exception as exc:  # noqa: BLE001
                self._logger.warn(f"Stop() raised: {exc}")

    def switch_mode(self, mode) -> None:
        if self._robot is None:
            raise RuntimeError("robot not connected")
        self._robot.SwitchMode(mode)

    def send_joint_position(
        self,
        positions: List[float],
        velocities: List[float],
        max_vel: List[float],
        max_acc: List[float],
    ) -> None:
        self._robot.SendJointPosition(positions, velocities, max_vel, max_acc)

    def send_cartesian_motion_force(
        self,
        pose: List[float],
        wrench: Optional[List[float]] = None,
        max_linear_vel: float = 0.5,
        max_angular_vel: float = 1.047,
    ) -> None:
        """Stream TCP pose [x,y,z,qw,qx,qy,qz] in world frame (RDK convention)."""
        if wrench is None:
            wrench = [0.0] * 6
        self._robot.SendCartesianMotionForce(
            pose, wrench, max_linear_vel=max_linear_vel, max_angular_vel=max_angular_vel
        )

    def execute_primitive_name(self, name: str) -> None:
        self.switch_mode(self._flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION)
        self._robot.ExecutePrimitive(name, {}, True)

    def states(self):
        return self._robot.states()

    def status_snapshot(self) -> Tuple[bool, bool, bool, str, str]:
        """Return fault, operational, enabled-ish, mode_str, detail."""
        if self._robot is None:
            return True, False, False, "DISCONNECTED", "no robot handle"
        fault = bool(self._robot.fault())
        operational = bool(self._robot.operational())
        mode = mode_to_str(self._robot.mode())
        try:
            op_stat = self._robot.operational_status()
            detail = str(op_stat).split(".")[-1]
        except Exception:  # noqa: BLE001
            detail = ""
        enabled = operational and not fault
        return fault, operational, enabled, mode, detail
