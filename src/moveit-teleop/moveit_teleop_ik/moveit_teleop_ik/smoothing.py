"""Output shaping for the IK command stream.

Two independent stages, both pure and both optional:

``WeightedMovingAverage``
    xr_teleoperate's post-solve filter. The solver is already smooth frame to
    frame thanks to its smoothness cost, but a short average takes the edge off
    residual solver chatter without adding meaningful lag.

``JointRateLimiter``
    Caps each joint's step at what the URDF says the joint can actually do.
    The solver has no notion of time -- it returns a configuration, not a
    trajectory -- so nothing else stops it asking for a jump the hardware
    cannot follow. On mock hardware such a jump is simply executed, which
    looks like a teleport.
"""

from __future__ import annotations

from collections import deque

import numpy as np

#: xr_teleoperate's weights: most recent sample dominates, three older ones
#: taper off. Newest first.
DEFAULT_WEIGHTS = (0.4, 0.3, 0.2, 0.1)


class WeightedMovingAverage:
    """Weighted average over the last N samples, newest first."""

    def __init__(self, weights=DEFAULT_WEIGHTS):
        weights = np.asarray(weights, float)
        if weights.ndim != 1 or weights.size < 1:
            raise ValueError('weights must be a non-empty 1-D sequence')
        if np.any(weights < 0):
            raise ValueError('weights must be non-negative')
        total = weights.sum()
        if total <= 0:
            raise ValueError('weights must sum to something positive')
        self.weights = weights / total
        self._history = deque(maxlen=len(self.weights))

    def reset(self) -> None:
        """Drop the history. Call whenever the arm has been moved by something
        else, so stale samples cannot drag the first new command backwards."""
        self._history.clear()

    def __call__(self, sample) -> np.ndarray:
        sample = np.asarray(sample, float)
        self._history.appendleft(sample.copy())
        # Renormalise over however many samples exist, so the filter is
        # correct from the very first call rather than biased toward zero.
        active = self.weights[:len(self._history)]
        active = active / active.sum()
        return np.sum([w * s for w, s in zip(active, self._history)], axis=0)


class JointRateLimiter:
    """Limit per-joint change per update to ``velocity_limits * dt``.

    Velocity only. An acceleration limit was tried here and removed: ramping
    velocity without braking in anticipation of arrival overshoots and rings --
    measured, a step to 0.05 rad overshot by 44 mrad, crossed the target three
    times and settled at 0.085. On the arm that reads as the tool oscillating
    while the operator holds the clutch still.

    Doing it properly means a jerk-limited online trajectory generator that
    plans the deceleration, i.e. Ruckig (installed as ros-jazzy-ruckig 0.9.2).
    That is the route if acceleration limiting is ever actually needed; a naive
    clamp is worse than none.
    """

    def __init__(self, velocity_limits, scale: float = 1.0):
        self.velocity_limits = np.asarray(velocity_limits, float)
        if np.any(self.velocity_limits <= 0):
            raise ValueError('velocity limits must be positive')
        if not 0.0 < scale <= 1.0:
            raise ValueError('scale must be in (0, 1]')
        self.scale = float(scale)
        self._previous = None

    def reset(self, q=None) -> None:
        self._previous = None if q is None else np.asarray(q, float).copy()

    def __call__(self, q, dt: float) -> np.ndarray:
        q = np.asarray(q, float)
        if self._previous is None or dt <= 0:
            self._previous = q.copy()
            return q.copy()
        step = np.abs(self.velocity_limits) * self.scale * dt
        limited = np.clip(q, self._previous - step, self._previous + step)
        self._previous = limited.copy()
        return limited

    def would_limit(self, q, dt: float) -> bool:
        """Whether the last call clipped anything -- worth surfacing, because
        persistent rate limiting means the operator is outrunning the arm."""
        if self._previous is None or dt <= 0:
            return False
        step = np.abs(self.velocity_limits) * self.scale * dt
        return bool(np.any(np.abs(np.asarray(q, float) - self._previous) > step + 1e-12))
