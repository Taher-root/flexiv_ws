"""Find recorded episodes and turn one into a scrubbable timeline.

Deliberately free of rclpy so the indexing and seeking logic can be tested
without a ROS graph. The node in :mod:`episode_player` adds publishing and the
service; everything about WHAT to play is here.

Playback resamples through the same :class:`~moveit_teleop_record.reader.Episode`
machinery the exporter uses, rather than replaying the bag at its native rate.
That is the point: what you watch is what a training sample would contain,
including the frames that were DROPPED because a required stream was stale. An
episode that looks fine in `ros2 bag play` and has holes at export time is
exactly the failure worth being able to see.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


@dataclass
class EpisodeSummary:
    """What a UI needs to list an episode without opening its bag."""

    name: str
    path: Path
    task: str
    started_utc: str
    end_reason: str
    halted: bool

    @property
    def sort_key(self):
        # started_utc is ISO-8601 Zulu, so lexicographic order is chronological.
        return (self.started_utc, self.name)


def discover(directory) -> list:
    """Episode summaries under *directory*, newest first.

    A directory without an ``episode.json`` is skipped rather than raising:
    the episodes directory is also where partially written and discarded
    episodes land, and one bad entry must not hide every good one.
    """
    root = Path(directory).expanduser()
    if not root.is_dir():
        return []

    found = []
    for child in sorted(root.iterdir()):
        sidecar = child / 'episode.json'
        if not child.is_dir() or not sidecar.exists():
            continue
        try:
            meta = json.loads(sidecar.read_text())
        except (json.JSONDecodeError, OSError):
            continue
        found.append(EpisodeSummary(
            name=child.name,
            path=child,
            task=meta.get('task', ''),
            started_utc=meta.get('started_utc', ''),
            end_reason=meta.get('end_reason', ''),
            halted=bool(meta.get('halted_during_episode', False)),
        ))
    found.sort(key=lambda s: s.sort_key, reverse=True)
    return found


# TeleopStatus.mode values, duplicated as plain ints so this module does not
# need the message package. Guarded by a test that compares them.
MODE_TELEOP = 2


@dataclass
class Timeline:
    """A loaded episode, indexed for random access.

    ``frames`` is materialised rather than streamed because scrubbing is the
    whole point: a slider that can only go forwards is not a review tool.
    Episodes are seconds-to-minutes of a handful of small topics, so the memory
    cost is trivial next to the usability.
    """

    name: str
    task: str
    frames: list          # list of moveit_teleop_record.reader.Frame
    mode_times: list      # seconds from start
    mode_values: list

    @property
    def count(self) -> int:
        return len(self.frames)

    @property
    def duration(self) -> float:
        return self.frames[-1].timestamp if self.frames else 0.0

    def clamp(self, index: int) -> int:
        if not self.frames:
            return 0
        return max(0, min(int(index), len(self.frames) - 1))

    def at(self, index: int):
        return self.frames[self.clamp(index)] if self.frames else None


def mode_timeline(meta: dict, started: str) -> tuple:
    """Convert the sidecar's UTC mode transitions to seconds from start.

    Returned so a UI can shade the parts of an episode that were actually
    engaged. Entries that cannot be parsed are dropped rather than guessed at.
    """
    from datetime import datetime

    def parse(text):
        try:
            return datetime.fromisoformat(text.replace('Z', '+00:00'))
        except (ValueError, AttributeError):
            return None

    origin = parse(started)
    if origin is None:
        return [], []

    times, values = [], []
    for entry in meta.get('mode_timeline', []) or []:
        stamp = parse(entry.get('utc'))
        if stamp is None:
            continue
        times.append((stamp - origin).total_seconds())
        values.append(int(entry.get('mode', 0)))
    return times, values


def load(summary: EpisodeSummary, rate_hz=None) -> Timeline:
    """Open an episode and resample it into a Timeline."""
    from moveit_teleop_record.reader import Episode

    episode = Episode(summary.path)
    frames = list(episode.frames(rate_hz=rate_hz))
    times, values = mode_timeline(episode.meta, summary.started_utc)
    return Timeline(name=summary.name, task=summary.task, frames=frames,
                    mode_times=times, mode_values=values)
