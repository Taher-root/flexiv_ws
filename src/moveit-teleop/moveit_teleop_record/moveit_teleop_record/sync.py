"""Time alignment for exporting episodes to training datasets.

Pure numpy: no ROS, no bag reading, no LeRobot. Everything here is a function
of timestamps and arrays, which is what makes it testable -- and time alignment
is the part of an export that fails silently, so it is the part that most needs
testing.

The rules, and the measurements behind them
-------------------------------------------
**Align on capture time (header.stamp), never on receive time.** Measured on a
Piper episode with the Gemini 335L: the camera's receive time trails its
capture time by 30.6 +/- 0.7 ms, while the arm's joint feedback trails by only
4.7 ms. Pairing on receive time would give every image the arm state from
~26 ms after the picture was taken. Cross-correlating image motion against
joint speed confirmed it independently: 34 ms off on receive time, within
10 ms on header time.

**The camera drives the timeline.** Each exported frame is one real camera
frame, and the joint streams (200 Hz) are interpolated to that frame's capture
time. Resampling images instead would invent frames that were never captured.

**Deduplicate before interpolating.** The Piper driver republishes a joint
sample when no new CAN frame has arrived, repeating its header stamp -- 0.8% of
samples in that episode. Interpolating across a zero-width interval divides by
zero.

**Refuse rather than repair.** A frame with no joint sample nearby, or a hole
in the camera stream, fails the episode with a reason. A training set that
quietly contains a held value across a dropout teaches the policy that the arm
did not move.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

#: TeleopStatus.TELEOP -- the operator is engaged and driving the arm.
TELEOP = 2


class SyncError(ValueError):
    """An episode cannot be aligned without inventing data."""


@dataclass
class Series:
    """One signal on its own clock: strictly increasing times, one row each."""

    t: np.ndarray     # (N,) float64 seconds
    v: np.ndarray     # (N, D)

    @classmethod
    def from_samples(cls, times, values) -> tuple:
        """Build a Series, returning ``(series, repeated_stamps_dropped)``.

        Sorted stably, then only the LAST sample of each repeated stamp is kept:
        a driver that republishes an unchanged sample stamps it with the time of
        the original, so the later copy carries no new information and the
        earlier one is the true measurement at that time either way.
        """
        t = np.asarray(times, dtype=float).reshape(-1)
        v = np.asarray(values, dtype=float)
        if v.ndim == 1:
            v = v[:, None]
        if len(t) != len(v):
            raise ValueError(f'{len(t)} times for {len(v)} values')
        if len(t) == 0:
            raise SyncError('series is empty')
        order = np.argsort(t, kind='stable')
        t, v = t[order], v[order]
        keep = np.ones(len(t), dtype=bool)
        keep[:-1] = np.diff(t) > 0
        return cls(t[keep], v[keep]), int((~keep).sum())


def sample(series: Series, query, max_gap: float) -> tuple:
    """Linearly interpolate *series* at *query* times.

    Returns ``(values, ok)``. ``ok`` is False for a query outside the series,
    or one whose two bracketing samples are more than *max_gap* apart -- that is
    a dropout, and a value interpolated across it is a guess.

    Linear is right for joint positions: at 200 Hz the arm moves a fraction of a
    degree between samples, so the error is far below encoder noise.
    """
    q = np.asarray(query, dtype=float).reshape(-1)
    t = series.t
    i = np.searchsorted(t, q, side='right')          # t[i-1] <= q < t[i]
    lo = np.clip(i - 1, 0, len(t) - 1)
    hi = np.clip(i, 0, len(t) - 1)
    gap = t[hi] - t[lo]
    ok = (q >= t[0]) & (q <= t[-1]) & (gap <= max_gap)
    safe = np.where(gap > 0, gap, 1.0)
    w = np.where(gap > 0, (q - t[lo]) / safe, 0.0)
    values = series.v[lo] + (series.v[hi] - series.v[lo]) * w[:, None]
    return values, ok


def active_span(mode_times, modes, active: int = TELEOP):
    """``(start, end)`` of the time the operator was driving, or None.

    From the first status that reports *active* to the last one. Pauses inside
    that span -- the clutch released while the operator repositions a hand --
    are kept: the arm holds still through them, and cutting them would splice
    two separate moments into one continuous video.

    Everything outside is dropped, and it is not demonstration: the idle before
    the operator engages, and in particular a HOME at the end, where the
    PLANNER moves the arm and the recorded action is not the operator's.
    """
    t = np.asarray(mode_times, dtype=float)
    m = np.asarray(modes)
    idx = np.flatnonzero(m == active)
    if len(idx) == 0:
        return None
    return float(t[idx[0]]), float(t[idx[-1]])


def camera_rate(stamps) -> float:
    """The camera's true frame rate, from the median capture interval."""
    d = np.diff(np.asarray(stamps, dtype=float))
    d = d[d > 0]
    if len(d) == 0:
        raise SyncError('need at least two distinct camera frames')
    return float(1.0 / np.median(d))


def frame_gaps(stamps, fps: float, tolerance: float = 0.5) -> list:
    """``[(frame_index, frames_missing), ...]`` wherever the camera skipped.

    A gap is an interval longer than ``(1 + tolerance)`` frame periods.
    """
    d = np.diff(np.asarray(stamps, dtype=float))
    period = 1.0 / fps
    bad = np.flatnonzero(d > (1.0 + tolerance) * period)
    return [(int(i + 1), max(1, int(round(d[i] / period)) - 1)) for i in bad]


def choose_fps(true_rate: float, max_mismatch: float = 0.02) -> int:
    """The integer dataset fps for a camera running at *true_rate*.

    LeRobot requires an integer fps and timestamps exactly on ``index / fps``.
    Exported timestamps are therefore nominal, and they drift from wall time by
    the rate mismatch -- 0.1% for a 335L asked for 15 fps, which is 25 ms over a
    25 s episode. A mismatch beyond *max_mismatch* means the camera is not
    running at the rate anyone thinks it is, and is refused.
    """
    fps = int(round(true_rate))
    if fps <= 0:
        raise SyncError(f'camera rate {true_rate:.3f} Hz rounds to no fps')
    mismatch = abs(true_rate - fps) / fps
    if mismatch > max_mismatch:
        raise SyncError(
            f'camera runs at {true_rate:.3f} Hz, {100 * mismatch:.1f}% off the '
            f'nearest integer fps {fps}; exported timestamps would drift from '
            'real time. Set the camera to a whole frame rate.')
    return fps


def motion_correlation(frame_times, frame_motion, series: Series,
                       lags=None, window=None) -> tuple:
    """Correlation of image motion with joint speed, at every candidate shift.

    For a camera mounted on the arm, how much the image changes between two
    frames is driven by how fast the arm moved in that interval:

        frame_motion[k]  ~  mean joint speed over
                            [frame_times[k] + L, frame_times[k + 1] + L]

    Returns ``(lags, r, n_intervals)``: Pearson r at each shift L. A positive L
    means joint motion shows up in the images L seconds before the joint
    timestamps say it happened -- the joint stamps run late.
    """
    ft = np.asarray(frame_times, dtype=float)
    motion = np.asarray(frame_motion, dtype=float)
    if len(motion) != len(ft) - 1:
        raise ValueError('frame_motion must have one entry per frame interval')
    lags = np.arange(-0.200, 0.2001, 0.002) if lags is None else np.asarray(lags, float)

    t = series.t
    speed = np.linalg.norm(np.gradient(series.v, t, axis=0), axis=1)
    area = np.concatenate([[0.0], np.cumsum(0.5 * (speed[1:] + speed[:-1])
                                              * np.diff(t))])
    mask = np.ones(len(motion), dtype=bool)
    if window is not None:
        mask &= (ft[1:] >= window[0]) & (ft[:-1] <= window[1])

    r = np.full(len(lags), np.nan)
    counts = np.zeros(len(lags), dtype=int)
    for j, lag in enumerate(lags):
        a, b = ft[:-1] + lag, ft[1:] + lag
        ok = mask & (a >= t[0]) & (b <= t[-1])
        if ok.sum() < 10:
            continue
        s = (np.interp(b[ok], t, area) - np.interp(a[ok], t, area)) / np.maximum(b[ok] - a[ok], 1e-9)
        if np.std(s) < 1e-12 or np.std(motion[ok]) < 1e-12:
            continue
        r[j] = float(np.corrcoef(motion[ok], s)[0, 1])
        counts[j] = int(ok.sum())
    return lags, r, int(counts.max()) if counts.any() else 0


def lag_verdict(lags, r, n: int, frame_period: float) -> dict:
    """Judge a correlation curve, honestly accounting for how flat it is.

    The peak of a correlation curve is only as meaningful as its sharpness.
    Measured on four Piper episodes at 15 fps, the curve is FLAT: every shift
    within an 84-136 ms band fits within 0.01 of the best, because arm speed
    changes slowly next to a 66 ms frame interval. One episode peaked at
    -100 ms while scoring 0.02 lower at 0 ms -- a third of the sampling error on
    r for 134 frames. Reading the peak alone called that episode misaligned;
    pooled over all four the best shift was -6 ms and 0 fit exactly as well.

    So the verdict uses a band: every shift whose r is within one standard error
    of the peak, ``(1 - r^2) / sqrt(n - 3)``.

      aligned        0 is inside the band -- no evidence of an offset
      MISALIGNED     the whole band lies more than half a frame from 0
      inconclusive   neither; or r < 0.3, too little arm-driven image motion
    """
    lags = np.asarray(lags, float)
    r = np.asarray(r, float)
    half = 0.5 * frame_period
    out = {'offset_ms': None, 'r': None, 'band_ms': None, 'intervals': int(n),
           'half_frame_ms': round(half * 1000, 1), 'verdict': 'inconclusive'}
    if not np.isfinite(r).any() or n < 10:
        out['verdict'] = 'inconclusive (not enough frames)'
        return out
    i = int(np.nanargmax(r))
    peak = float(r[i])
    out['offset_ms'] = round(float(lags[i]) * 1000, 1)
    out['r'] = round(peak, 3)
    if peak < 0.3:
        out['verdict'] = 'inconclusive (too little arm-driven image motion)'
        return out
    se = (1.0 - peak ** 2) / np.sqrt(max(n - 3, 1))
    band = lags[np.isfinite(r) & (r >= peak - se)]
    lo, hi = float(band.min()), float(band.max())
    out['band_ms'] = [round(lo * 1000), round(hi * 1000)]
    if lo <= 0.0 <= hi:
        out['verdict'] = 'aligned'
    elif lo > half or hi < -half:
        out['verdict'] = 'MISALIGNED'
    return out


def pool_curves(curves) -> tuple:
    """Combine per-episode ``(lags, r, n)`` into one curve, weighted by n.

    Same sensors and clocks in every episode, so the evidence adds up: four
    short episodes whose peaks wander individually pin the offset down together.
    """
    curves = [c for c in curves if c[2] > 0]
    if not curves:
        return None, None, 0
    lags = curves[0][0]
    stack = np.array([np.nan_to_num(c[1], nan=0.0) * c[2] for c in curves])
    n = sum(c[2] for c in curves)
    return lags, stack.sum(axis=0) / n, n
