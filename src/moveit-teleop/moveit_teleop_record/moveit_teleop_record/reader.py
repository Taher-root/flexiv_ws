"""Read episode bags and resample them onto a common timeline.

Sync happens here, at export, not during recording. Bags hold every message at
its native rate with its own timestamp; this turns that into evenly spaced
frames. The consequence worth having is that the same episodes can be
re-exported at a different rate, with a different interpolation rule, or with a
different set of streams, without re-recording anything.

The rule that matters: a frame is only emitted when EVERY required stream has a
sample within ``max_staleness`` of it. Silently carrying a stale value forward
across a gap produces training frames whose action never actually corresponded
to that observation, which is worse than having fewer frames.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


@dataclass
class Frame:
    """One resampled instant: ``{stream_name: message}`` plus its timestamp."""

    timestamp: float          # seconds since episode start
    index: int
    values: dict


class Episode:
    """One recorded episode: its bag, its sidecar, and its resampled frames."""

    def __init__(self, path):
        self.path = Path(path)
        sidecar = self.path / 'episode.json'
        if not sidecar.exists():
            raise FileNotFoundError(
                f'{self.path} has no episode.json; not a teleop episode')
        self.meta = json.loads(sidecar.read_text())
        self.streams = {s['name']: s for s in self.meta.get('streams', [])}
        self._by_topic = {s['topic']: s for s in self.meta.get('streams', [])}

    # ---- raw access -------------------------------------------------------

    def read_raw(self) -> dict:
        """Return ``{stream_name: [(t_seconds, message), ...]}``, unmodified."""
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(self.path), storage_id='mcap'),
            rosbag2_py.ConverterOptions('', ''))
        types = {t.name: t.type for t in reader.get_all_topics_and_types()}

        out = {}
        while reader.has_next():
            topic, data, stamp_ns = reader.read_next()
            stream = self._by_topic.get(topic)
            if stream is None:
                continue
            msg = deserialize_message(data, get_message(types[topic]))
            out.setdefault(stream['name'], []).append((stamp_ns * 1e-9, msg))
        for series in out.values():
            series.sort(key=lambda pair: pair[0])
        return out

    # ---- resampling -------------------------------------------------------

    def frames(self, rate_hz=None, max_staleness=None, required=None):
        """Yield :class:`Frame` objects on an evenly spaced timeline.

        ``required`` defaults to every stream the config marked required. A
        frame missing any of them is dropped, and the count is reported by
        :meth:`frame_stats`.
        """
        export = self.meta.get('export', {})
        rate_hz = float(rate_hz or export.get('rate_hz', 30.0))
        max_staleness = float(
            max_staleness if max_staleness is not None
            else export.get('max_staleness', 0.05))

        raw = self.read_raw()
        if not raw:
            return

        if required is None:
            required = [name for name, s in self.streams.items()
                        if s.get('required') and name in raw]
            if not required:
                required = list(raw)

        starts = [series[0][0] for series in raw.values() if series]
        ends = [series[-1][0] for series in raw.values() if series]
        # Start when every stream is live and stop when the first one ends:
        # extrapolating past a stream's own data is invention, not resampling.
        t0, t1 = max(starts), min(ends)
        if t1 <= t0:
            return

        times = {name: np.array([t for t, _ in series])
                 for name, series in raw.items()}

        self._dropped = 0
        index = 0
        step = 1.0 / rate_hz
        t = t0
        while t <= t1:
            values, ok = {}, True
            for name, series in raw.items():
                stamps = times[name]
                # Nearest sample, then reject if it is too far away. Nearest
                # rather than previous-only because at export time the future
                # is available and a sample 5 ms ahead is a better estimate
                # than one 30 ms behind.
                i = int(np.argmin(np.abs(stamps - t)))
                if abs(stamps[i] - t) > max_staleness:
                    if name in required:
                        ok = False
                        break
                    continue
                values[name] = series[i][1]
            if ok:
                yield Frame(timestamp=t - t0, index=index, values=values)
                index += 1
            else:
                self._dropped += 1
            t += step

    def frame_stats(self) -> dict:
        return {'dropped_frames': getattr(self, '_dropped', 0)}
