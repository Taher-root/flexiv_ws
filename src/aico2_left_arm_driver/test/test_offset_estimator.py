"""Unit tests for offset_estimator (no rclpy, no hardware required)."""

from types import SimpleNamespace

import pytest

from aico2_left_arm_driver.offset_estimator import (
    OffsetEstimate,
    OffsetTracker,
    estimate_offset,
)

WALL_ANCHOR = 1_700_000_000.0
TRUE_OFFSET = 13.0


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
        return SimpleNamespace(timestamp=self._ts)


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
        estimate_offset(lambda: SimpleNamespace(timestamp=0.0), n=0)


def test_estimate_offset_raises_if_device_never_ticks():
    stuck = SimpleNamespace(timestamp=1.0)
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


def test_to_ros_seconds_applies_offset():
    tracker = OffsetTracker(states_fn=lambda: None)
    tracker.apply_estimate(OffsetEstimate(offset_sec=13.0, spread_sec=0.0, n_transitions=1))
    assert tracker.to_ros_seconds(100.0) == pytest.approx(113.0)
