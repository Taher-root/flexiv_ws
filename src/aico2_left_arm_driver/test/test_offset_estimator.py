"""Unit tests for offset_estimator (no rclpy, no hardware required)."""

from types import SimpleNamespace

import pytest

from aico2_left_arm_driver.offset_estimator import (
    OffsetEstimate,
    OffsetTracker,
    device_time_seconds,
    estimate_offset,
)

WALL_ANCHOR = 1_700_000_000.0
TRUE_OFFSET = 13.0


def _to_raw_timestamp(seconds):
    """float seconds -> (sec, nanosec) int tuple, RobotStates.timestamp's real shape."""
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return sec, nanosec


class _FakeClock:
    """Deterministic stand-in for time.monotonic()/time.time() with a fixed link."""

    def __init__(self) -> None:
        self.mono = 0.0

    def monotonic(self) -> float:
        return self.mono

    def wall(self) -> float:
        return self.mono + WALL_ANCHOR

    def advance(self, dt: float) -> None:
        self.mono += dt


class _FakeRobot:
    """Emits a device timestamp that ticks every `tick` seconds of fake mono time."""

    def __init__(self, clock: _FakeClock, tick: float = 0.001, poll_cost: float = 2e-5) -> None:
        self._clock = clock
        self._tick = tick
        self._poll_cost = poll_cost
        self._next_tick_mono = tick
        self._ts = clock.wall() + TRUE_OFFSET

    def states(self):
        self._clock.advance(self._poll_cost)
        if self._clock.mono >= self._next_tick_mono:
            self._next_tick_mono += self._tick
            self._ts = self._clock.wall() + TRUE_OFFSET
        return SimpleNamespace(timestamp=_to_raw_timestamp(self._ts))


def test_device_time_seconds_decodes_sec_nanosec_tuple():
    assert device_time_seconds((10, 500_000_000)) == pytest.approx(10.5)
    assert device_time_seconds((1789742773, 292993000)) == pytest.approx(1789742773.292993)


def test_estimate_offset_recovers_known_offset():
    clock = _FakeClock()
    robot = _FakeRobot(clock)

    estimate = estimate_offset(
        robot.states,
        n=50,
        monotonic_fn=clock.monotonic,
        wall_fn=clock.wall,
    )

    assert estimate.n_transitions == 50
    assert abs(estimate.offset_sec - TRUE_OFFSET) < 1e-3
    assert estimate.spread_sec < 1e-3


def test_estimate_offset_rejects_non_positive_n():
    with pytest.raises(ValueError):
        estimate_offset(lambda: SimpleNamespace(timestamp=(0, 0)), n=0)


def test_estimate_offset_raises_if_device_never_ticks():
    stuck = SimpleNamespace(timestamp=(1, 0))
    with pytest.raises(RuntimeError):
        estimate_offset(lambda: stuck, n=5, max_polls=20)


def test_offset_tracker_adopts_first_estimate_outright():
    tracker = OffsetTracker(states_fn=lambda: None, slew_limit_sec=0.001)
    tracker.apply_estimate(OffsetEstimate(offset_sec=13.0, spread_sec=0.0, n_transitions=1))
    assert tracker.offset_sec == pytest.approx(13.0)


def test_offset_tracker_slews_large_jumps():
    tracker = OffsetTracker(states_fn=lambda: None, slew_limit_sec=0.001)
    tracker.apply_estimate(OffsetEstimate(offset_sec=0.0, spread_sec=0.0, n_transitions=1))

    tracker.apply_estimate(OffsetEstimate(offset_sec=1.0, spread_sec=0.0, n_transitions=1))

    assert tracker.offset_sec == pytest.approx(0.001)


def test_offset_tracker_does_not_overshoot_a_small_move():
    tracker = OffsetTracker(states_fn=lambda: None, slew_limit_sec=0.001)
    tracker.apply_estimate(OffsetEstimate(offset_sec=0.0, spread_sec=0.0, n_transitions=1))

    tracker.apply_estimate(OffsetEstimate(offset_sec=0.0005, spread_sec=0.0, n_transitions=1))

    assert tracker.offset_sec == pytest.approx(0.0005)


def test_offset_tracker_due_before_and_after_refresh_window():
    clock = {"t": 0.0}
    tracker = OffsetTracker(
        states_fn=lambda: None, refresh_sec=10.0, clock_fn=lambda: clock["t"]
    )
    assert tracker.due() is True

    tracker.apply_estimate(OffsetEstimate(offset_sec=1.0, spread_sec=0.0, n_transitions=1))
    assert tracker.due() is False

    clock["t"] = 10.0
    assert tracker.due() is True


def test_to_ros_seconds_subtracts_offset():
    # offset_sec = device_time - host_time (device ahead when positive), so
    # recovering host time from a device reading must SUBTRACT it. A device
    # clock 13s ahead reading "113" corresponds to host time "100".
    tracker = OffsetTracker(states_fn=lambda: None)
    tracker.apply_estimate(OffsetEstimate(offset_sec=13.0, spread_sec=0.0, n_transitions=1))
    assert tracker.to_ros_seconds((113, 0)) == pytest.approx(100.0)


def test_to_ros_seconds_matches_real_hardware_reading():
    # Regression check against the actual capture that caught the sign bug
    # (left arm, 2026-09-17): device_time - offset reproduced time.time()
    # at the moment of that reading to within float precision.
    tracker = OffsetTracker(states_fn=lambda: None)
    tracker.apply_estimate(
        OffsetEstimate(offset_sec=29123.436620235443, spread_sec=0.0, n_transitions=1)
    )
    assert tracker.to_ros_seconds((1789744375, 351014000)) == pytest.approx(
        1789715251.9143937, abs=1e-3
    )
