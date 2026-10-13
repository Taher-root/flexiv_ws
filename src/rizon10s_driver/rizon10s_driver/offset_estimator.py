"""Device-to-host clock offset estimation for Flexiv RDK state streams.

See joint_state_architecture.md sec 4.4 / 6. Pure Python, no rclpy
dependency, so it can be unit tested without a ROS environment or hardware.
"""

from __future__ import annotations

import statistics
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

RawTimestamp = Tuple[int, int]


def device_time_seconds(raw_timestamp: RawTimestamp) -> float:
    """Convert a raw RobotStates.timestamp to seconds.

    Confirmed 2026-09-17 against flexivrdk 1.9.0 on real hardware (see
    scripts/rdk_diag.py): RobotStates.timestamp is a (sec, nanosec) int
    pair — the same shape as ROS's own Time message — not a scalar. The
    device/host offset it revealed (~29123 s on the left arm) lines up with
    the ~29107 s measured for joint_state_architecture.md sec 1, so this is
    the same clock, correctly decoded.
    """
    sec, nanosec = raw_timestamp
    return float(sec) + float(nanosec) * 1e-9


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
    timeout_sec: float = 5.0,
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

    Bounded by wall-clock time, not a poll count: how many polls it takes to
    gather n transitions depends on the execution context. Confirmed on real
    hardware 2026-09-18: a bare single-threaded script reached ~800k polls/s,
    but the same loop running inside a live rclpy LifecycleNode (logging,
    threading, executor overhead) only reached ~700k polls/s — enough to
    make an earlier poll-COUNT cap (n*500) fall short by 15-25% every time,
    even though the device itself streamed fine throughout. n=200 at 1 kHz
    needs >=200ms; the 5 s default leaves generous headroom for a slower
    context without ever meaningfully delaying a real failure.

    monotonic_fn/wall_fn are injected so this is testable with a fake clock;
    in production they default to time.monotonic/time.time.
    """
    if n <= 0:
        raise ValueError("n must be positive")

    mono0 = monotonic_fn()
    wall0 = wall_fn()
    deadline = mono0 + timeout_sec

    def host_time_at(t_mono: float) -> float:
        return wall0 + (t_mono - mono0)

    samples: List[float] = []
    prev_ts: Optional[RawTimestamp] = None
    polls = 0
    while len(samples) < n:
        if monotonic_fn() >= deadline:
            raise RuntimeError(
                f"only {len(samples)}/{n} timestamp transitions seen in "
                f"{timeout_sec}s ({polls} polls); device may not be streaming"
            )
        polls += 1
        t0 = monotonic_fn()
        state = states_fn()
        t1 = monotonic_fn()
        # Keep the raw (sec, nanosec) tuple for the equality check — tuple
        # equality is exact, unlike comparing two floats derived from it.
        ts: RawTimestamp = getattr(state, timestamp_attr)
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

    Sign convention (matches estimate_offset): offset_sec = device_time -
    host_time, so a positive offset means the device clock reads ahead of
    the host. to_ros_seconds() must therefore SUBTRACT it to recover a
    host-equivalent time from a device timestamp. Confirmed against real
    hardware 2026-09-17 (left arm): offset_sec ~= +29123s (device ahead);
    device_time_seconds(reading) - offset_sec reproduced time.time() at
    the moment of that reading to 7 decimal places. An earlier version of
    this method added the offset instead, which would have doubled the
    drift to ~58000s and made every published stamp unusable by tf2 —
    caught here before it ever ran against hardware.
    """

    def __init__(
        self,
        states_fn: Callable[[], object],
        refresh_sec: float = 300.0,
        slew_limit_sec: float = 0.001,
        n_samples: int = 200,
        estimate_timeout_sec: float = 5.0,
        clock_fn: Callable[[], float] = time.monotonic,
    ) -> None:
        self._states_fn = states_fn
        self._refresh_sec = refresh_sec
        self._slew_limit = slew_limit_sec
        self._n_samples = n_samples
        self._estimate_timeout_sec = estimate_timeout_sec
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
        estimate = estimate_offset(
            self._states_fn, n=self._n_samples, timeout_sec=self._estimate_timeout_sec
        )
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

    def to_ros_seconds(self, device_timestamp: RawTimestamp) -> float:
        """Host/ROS-clock-equivalent seconds for a raw device timestamp."""
        return device_time_seconds(device_timestamp) - self._offset_sec
