"""Tests for output shaping."""

import numpy as np
import pytest

from moveit_teleop_ik.smoothing import (
    DEFAULT_WEIGHTS, JointRateLimiter, WeightedMovingAverage,
)


# ---- moving average --------------------------------------------------------

def test_constant_input_passes_through_unchanged():
    """A filter that biases a steady signal would add permanent error."""
    f = WeightedMovingAverage()
    for _ in range(10):
        assert np.allclose(f([1.0, -2.0, 0.5]), [1.0, -2.0, 0.5])


def test_correct_from_the_very_first_sample():
    """Weights are renormalised over the samples that exist, so a cold filter
    does not drag the first command toward zero."""
    f = WeightedMovingAverage()
    assert np.allclose(f([1.0]), [1.0]), 'first sample was attenuated'


def test_applies_the_documented_weights_once_full():
    f = WeightedMovingAverage()
    for value in (1.0, 2.0, 3.0, 4.0):        # oldest first
        out = f([value])
    expected = sum(w * v for w, v in zip(DEFAULT_WEIGHTS, [4.0, 3.0, 2.0, 1.0]))
    assert out[0] == pytest.approx(expected)


def test_lags_a_step_then_converges():
    f = WeightedMovingAverage()
    f([0.0])
    first = f([1.0])[0]
    assert 0.0 < first < 1.0, 'a step should be attenuated, not passed or blocked'
    for _ in range(10):
        out = f([1.0])
    assert out[0] == pytest.approx(1.0)


def test_reset_forgets_history():
    f = WeightedMovingAverage()
    for _ in range(4):
        f([10.0])
    f.reset()
    assert f([0.0])[0] == pytest.approx(0.0), 'stale history leaked after reset'


def test_rejects_bad_weights():
    for bad in ([], [0.0, 0.0], [-1.0, 2.0]):
        with pytest.raises(ValueError):
            WeightedMovingAverage(bad)


# ---- rate limiter ----------------------------------------------------------

def test_limits_a_step_to_the_velocity_limit():
    lim = JointRateLimiter([1.0, 2.0])
    lim(np.zeros(2), 0.01)                     # seed
    out = lim([10.0, 10.0], dt=0.1)
    assert np.allclose(out, [0.1, 0.2]), 'step exceeded velocity * dt'


def test_passes_motion_within_the_limit():
    lim = JointRateLimiter([5.0])
    lim([0.0], 0.01)
    assert np.allclose(lim([0.01], dt=0.01), [0.01])


def test_converges_over_successive_calls():
    lim = JointRateLimiter([1.0])
    lim([0.0], 0.01)
    for _ in range(200):
        out = lim([0.5], dt=0.01)
    assert out[0] == pytest.approx(0.5)


def test_scale_tightens_the_limit():
    lim = JointRateLimiter([1.0], scale=0.5)
    lim([0.0], 0.01)
    assert np.allclose(lim([10.0], dt=0.1), [0.05])


def test_reports_when_it_is_clipping():
    """Persistent clipping means the operator is outrunning the arm; that is
    worth surfacing rather than hiding."""
    lim = JointRateLimiter([1.0])
    lim([0.0], 0.01)
    assert lim.would_limit([10.0], dt=0.01)
    lim([0.005], dt=0.01)
    assert not lim.would_limit([0.006], dt=0.01)


def test_first_call_is_not_limited():
    """Nothing to limit against yet -- clamping toward zero would be wrong."""
    lim = JointRateLimiter([1.0])
    assert np.allclose(lim([3.0], dt=0.01), [3.0])


def test_rejects_bad_configuration():
    with pytest.raises(ValueError):
        JointRateLimiter([0.0])
    with pytest.raises(ValueError):
        JointRateLimiter([1.0], scale=0.0)


def test_step_response_does_not_overshoot_or_ring():
    """The regression that reached the robot.

    An acceleration-limited version of this limiter ramped velocity without
    braking for arrival: a step to 0.05 rad overshot by 44 mrad, crossed the
    target three times and settled at 0.085. On the arm that showed up as the
    tool oscillating while the operator simply held the clutch still.

    Any future limiter must pass this: monotonic approach, no overshoot, exact
    convergence.
    """
    dt = 1.0 / 90.0
    target = 0.05
    lim = JointRateLimiter([0.5], scale=1.0)
    lim([0.0], dt)

    out = np.array([lim([target], dt)[0] for _ in range(400)])

    assert out.max() <= target + 1e-12, (
        f'overshot by {(out.max() - target) * 1000:.2f} mrad')
    crossings = int(np.sum(np.diff(np.sign(out - target)) != 0))
    assert crossings <= 1, f'rang about the target {crossings} times'
    assert np.all(np.diff(out) >= -1e-12), 'approach was not monotonic'
    assert out[-1] == pytest.approx(target, abs=1e-9), (
        f'settled at {out[-1]:.5f} instead of {target}')


def test_settles_within_a_sensible_time():
    """A limiter that is correct but glacial is also unusable."""
    dt = 1.0 / 90.0
    lim = JointRateLimiter([1.0], scale=1.0)
    lim([0.0], dt)
    out = np.array([lim([0.05], dt)[0] for _ in range(400)])
    frames = int(np.argmax(out >= 0.05 - 1e-9))
    assert 0 < frames < 30, f'took {frames} frames ({frames * dt:.2f} s)'
