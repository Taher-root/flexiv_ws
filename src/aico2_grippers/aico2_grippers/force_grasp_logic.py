"""Map VR trigger [0,1] to open vs force-grasp commands with hysteresis."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, Optional, Tuple

CommandKind = Literal["open", "grasp"]


@dataclass
class ForceGraspParams:
    open_threshold: float = 0.08
    grasp_threshold: float = 0.15
    max_force_fraction: float = 0.75
    min_resend_force_delta: float = 3.0


class ForceGraspController:
    """Front trigger -> open (Move) or proportional Grasp(-force)."""

    def __init__(self, params: ForceGraspParams) -> None:
        self.p = params
        self._state: Optional[CommandKind] = None
        self._last_force: Optional[float] = None

    def reset(self) -> None:
        self._state = None
        self._last_force = None

    def update(
        self, trigger: float, max_force_n: float
    ) -> Optional[Tuple[CommandKind, float]]:
        """Return a new command or None if unchanged (debounced)."""
        trigger = max(0.0, min(1.0, float(trigger)))
        max_force_n = max(1.0, float(max_force_n))

        if trigger <= self.p.open_threshold:
            if self._state == "open":
                return None
            self._state = "open"
            self._last_force = None
            return ("open", 0.0)

        if trigger >= self.p.grasp_threshold:
            force = -trigger * self.p.max_force_fraction * max_force_n
            if (
                self._state == "grasp"
                and self._last_force is not None
                and abs(force - self._last_force) < self.p.min_resend_force_delta
            ):
                return None
            self._state = "grasp"
            self._last_force = force
            return ("grasp", force)

        return None
