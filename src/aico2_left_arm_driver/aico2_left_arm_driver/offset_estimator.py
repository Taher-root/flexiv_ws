"""Device-to-host clock offset estimation for Flexiv RDK state streams.

See joint_state_architecture.md sec 4.4 / 6. Pure Python, no rclpy
dependency, so it can be unit tested without a ROS environment or hardware.
"""

from __future__ import annotations

import statistics
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence


def device_time_seconds(raw_timestamp: float) -> float:
    """Convert a raw RobotStates.timestamp to seconds.

    flexivrdk 1.9 does not document the timestamp's unit or epoch. The
    baseline measurements in joint_state_architecture.md sec 1 treated
    successive diffs as seconds directly (1000.2 us cadence came out right
    that way), so this is the identity conversion until checked against
    real hardware. Change this in one place if that assumption is wrong.
    """
    return float(raw_timestamp)


@dataclass(frozen=True)
class OffsetEstimate:
    offset_sec: float
    spread_sec: float
    n_transitions: int


def _iqr(values: Sequence[float]) -> float:
    data = sorted(values)
    n = len(data)
    if n < 2:
        return 0.0
    q1 = statistics.median(data[: n // 2])
    q3 = statistics.median(data[(n + 1) // 2 :])
    return q3 - q1


def estimate_offset(
    states_fn: Callable[[], object],
    n: int = 200,
    max_polls: Optional[int] = None,
    timestamp_attr: str = "timestamp",
    monotonic_fn: Callable[[], float] = time.monotonic,
    wall_fn: Callable[[], float] = time.time,
) -> OffsetEstimate:
    """Bracket the device-to-host clock offset by catching timestamp transitions.

    Polls states_fn() as fast as possible and records
    device_time - host_time each time the device timestamp changes,
    bracketing the transition instant to the poll interval (microseconds)
    rather than the ~1 ms sample interval. That distinction matters: the
    291 us offset jitter measured in sec 1 was itself sampled at 20-50 Hz
    against the 1 kHz device stream, so it folds in up to 1 ms of phase
    quantisation. This is the "do it properly" version described in sec 6.

    monotonic_fn/wall_fn are injected so this is testable with a fake clock;
    in production they default to time.monotonic/time.time.
    """
    if n <= 0:
        raise ValueError("n must be positive")
    limit = max_polls if max_polls is not None else n * 500

    mono0 = monotonic_fn()
    wall0 = wall_fn()

    def host_time_at(t_mono: float) -> float:
        return wall0 + (t_mono - mono0)

    samples: List[float] = []
    prev_ts: Optional[float] = None
    polls = 0
    while len(samples) < n:
        if polls >= limit:
            raise RuntimeError(
                f"only {len(samples)}/{n} timestamp transitions seen in "
                f"{polls} polls; device may not be streaming"
            )
        polls += 1
        t0 = monotonic_fn()
        state = states_fn()
        t1 = monotonic_fn()
        ts = float(getattr(state, timestamp_attr))
        if prev_ts is not None and ts != prev_ts:
            device_t = device_time_seconds(ts)
            samples.append(device_t - host_time_at((t0 + t1) / 2.0))
        prev_ts = ts

    return OffsetEstimate(
        offset_sec=statistics.median(samples),
        spread_sec=_iqr(samples),
        n_transitions=len(samples),
    )


class OffsetTracker:
    """Per-controller offset estimate with periodic, slew-limited refresh.

    One instance per RDK session — sec 4.4: "The two arms differ by 13 s.
    Do not share one estimate between them." A step in header.stamp can
    push TF backwards in time and make tf2 discard its buffer, so refreshes
    move the applied offset by at most slew_limit_sec per call rather than
    jumping straight to the new estimate.
    """

    def __init__(
        self,
        states_fn: Callable[[], object],
        refresh_sec: float = 300.0,
        slew_limit_sec: float = 0.001,
        n_samples: int = 200,
        clock_fn: Callable[[], float] = time.monotonic,
    ) -> None:
        self._states_fn = states_fn
        self._refresh_sec = refresh_sec
        self._slew_limit = slew_limit_sec
        self._n_samples = n_samples
        self._clock_fn = clock_fn
        self._offset_sec = 0.0
        self._spread_sec = 0.0
        self._last_refresh: Optional[float] = None

    @property
    def offset_sec(self) -> float:
        return self._offset_sec

    @property
    def spread_sec(self) -> float:
        return self._spread_sec

    @property
    def last_refresh(self) -> Optional[float]:
        return self._last_refresh

    def due(self) -> bool:
        return self._last_refresh is None or (
            self._clock_fn() - self._last_refresh >= self._refresh_sec
        )

    def refresh(self) -> OffsetEstimate:
        estimate = estimate_offset(self._states_fn, n=self._n_samples)
        self.apply_estimate(estimate)
        return estimate

    def apply_estimate(self, estimate: OffsetEstimate) -> None:
        """Fold a new OffsetEstimate in, slew-limited after the first one."""
        if self._last_refresh is None:
            # Nothing to slew from yet; adopt the first estimate outright.
            self._offset_sec = estimate.offset_sec
        else:
            step = estimate.offset_sec - self._offset_sec
            clamped = max(-self._slew_limit, min(self._slew_limit, step))
            self._offset_sec += clamped
        self._spread_sec = estimate.spread_sec
        self._last_refresh = self._clock_fn()

    def to_ros_seconds(self, device_timestamp: float) -> float:
        return device_time_seconds(device_timestamp) + self._offset_sec
