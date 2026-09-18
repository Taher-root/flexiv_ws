"""Thin flexivrdk.Gripper wrapper (shared robot handle with arm driver)."""

from __future__ import annotations

import time
from typing import Optional

from rclpy.impl.rcutils_logger import RcutilsLogger


class FlexivGripper:
    """Enable/init and command one Flexiv gripper on an existing RDK Robot."""

    def __init__(
        self,
        robot,
        flexivrdk,
        gripper_name: str,
        logger: RcutilsLogger,
        open_vel_fraction: float = 0.5,
        open_force_fraction: float = 0.5,
    ) -> None:
        self._robot = robot
        self._rdk = flexivrdk
        self._name = gripper_name
        self._logger = logger
        self._open_vel_frac = open_vel_fraction
        self._open_force_frac = open_force_fraction
        self._gripper = None
        self._min_width = 0.0
        self._max_width = 0.08
        self._max_force = 30.0

    @property
    def ready(self) -> bool:
        return self._gripper is not None

    @property
    def max_force(self) -> float:
        return self._max_force

    @property
    def width(self) -> float:
        if not self.ready:
            return 0.0
        return float(self._gripper.states().width)

    def init(self) -> None:
        if self.ready:
            return
        mode = str(self._robot.mode()).split(".")[-1]
        if mode != "IDLE":
            self._robot.Stop()
            time.sleep(0.05)
            self._robot.SwitchMode(self._rdk.Mode.IDLE)
        self._gripper = self._rdk.Gripper(self._robot)
        self._gripper.Enable(self._name)
        tool = self._rdk.Tool(self._robot)
        tool.Switch(self._name)
        params = self._gripper.params()
        self._min_width = float(params.min_width)
        self._max_width = float(params.max_width)
        self._max_force = float(params.max_force)
        self._gripper.Init()
        for _ in range(50):
            if not self._gripper.states().is_moving:
                break
            time.sleep(0.1)
        self._logger.info(
            f"gripper '{self._name}' ready "
            f"(width {self._min_width:.3f}..{self._max_width:.3f} m, "
            f"max_force {self._max_force:.1f} N)"
        )

    def open(self) -> None:
        if not self.ready:
            raise RuntimeError("gripper not initialized")
        params = self._gripper.params()
        vel = self._open_vel_frac * float(params.max_vel)
        force = self._open_force_frac * float(params.max_force)
        self._gripper.Move(self._max_width, vel, force)

    def grasp(self, force_n: float) -> None:
        if not self.ready:
            raise RuntimeError("gripper not initialized")
        self._gripper.Grasp(float(force_n))
