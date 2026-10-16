"""Episode bag -> time-aligned frames, ready for a dataset writer.

The ROS half of the export. It reads a recorded episode on CAPTURE time,
aligns every stream to the camera, and writes a small self-describing bundle:

    bundle/
      meta.json          task, fps, joint names, sync report, trim
      arrays.npz         state (N, J), action (N, J), capture_t (N,)
      <camera>/000000.jpg  the recorded JPEG bytes of each kept frame

Nothing here imports LeRobot or torch, and the writer that consumes the bundle
imports nothing from ROS. That split is deliberate: ROS's Python and LeRobot's
pinned dependencies do not coexist in one interpreter without one of them
breaking, and each half is easier to test when it cannot reach the other.

Joints are matched by NAME, never by position in the message, and the dataset's
joint order is the ROBOT MODEL's (see robot_model.py), not a driver's. The
Piper's measured state lists ``joint1..joint6, gripper`` while its command stream
lists ``gripper, joint1..joint6`` -- pairing by index would silently make the
first action element the gripper and the first state element joint1.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .sync import (
    Series, SyncError, active_span, camera_rate, choose_fps, frame_gaps,
    lag_verdict, motion_correlation, sample,
)


@dataclass
class ExportSpec:
    """What to take from an episode, by the stream NAMES in its sidecar."""

    #: ``[{'name', 'joints', 'units', ...}, ...]`` from RobotModel.describe()
    #: -- the dataset's joint layout, derived from the SRDF. Required.
    groups: list = None
    #: Replaces every episode's recorded task string when set.
    task: str = None
    state_stream: str = 'joint_states'
    #: Commanded joints. Absent on mock hardware, where the command IS the
    #: state and the recorder stored the topic once as state_stream.
    action_stream: str = 'joint_commands'
    mode_stream: str = 'teleop_status'
    #: None -> every stream the sidecar gives role 'camera'. The first one
    #: drives the timeline.
    cameras: list = None
    #: 'teleop' keeps the span the operator was driving; 'none' keeps it all.
    trim: str = 'teleop'
    #: A frame whose nearest joint samples are further apart than this has no
    #: trustworthy state. Healthy 200 Hz feedback brackets every frame within
    #: ~5 ms (worst 10.9 ms measured).
    max_joint_gap: float = 0.05
    #: Added to camera capture times before sampling joints. 0 trusts the
    #: header stamps, which the motion cross-correlation measured at +10 ms.
    camera_offset: float = 0.0


@dataclass
class Bundle:
    source: str
    task: str
    fps: int
    state_names: list
    action_names: list
    cameras: list
    groups: list
    capture_t: np.ndarray
    state: np.ndarray
    action: np.ndarray
    jpegs: dict                   # camera -> [bytes, ...]
    report: dict = field(default_factory=dict)

    def __len__(self) -> int:
        return len(self.capture_t)


# ---- joints ---------------------------------------------------------------

def positions_by_name(msg, names) -> list:
    """Positions of *names*, in that order, or None if any is missing."""
    lookup = dict(zip(msg.name, msg.position))
    if not all(n in lookup for n in names):
        return None
    return [float(lookup[n]) for n in names]


# ---- reading --------------------------------------------------------------

def _stamp(msg) -> float:
    h = getattr(msg, 'header', None)
    if h is None:
        return float('nan')
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def read_streams(path: Path, topics: dict) -> dict:
    """``{stream_name: [(capture_t, recv_t, msg), ...]}`` for *topics*.

    *topics* maps stream name -> topic. Only those topics are deserialised.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    storage = 'mcap' if any(Path(path).glob('*.mcap')) else ''
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    by_topic = {topic: name for name, topic in topics.items()}
    out = {name: [] for name in topics}
    while reader.has_next():
        topic, raw, recv_ns = reader.read_next()
        name = by_topic.get(topic)
        if name is None:
            continue
        msg = deserialize_message(raw, get_message(types[topic]))
        out[name].append((_stamp(msg), recv_ns * 1e-9, msg))
    return out


# ---- the alignment --------------------------------------------------------

def extract(episode_dir, spec: ExportSpec = None) -> Bundle:
    """Align one recorded episode. Raises SyncError with a reason if it can't."""
    spec = spec or ExportSpec()
    if not spec.groups:
        raise ValueError('ExportSpec.groups is required: load it with '
                         'robot_model.load_robot_model(...).describe()')
    episode_dir = Path(episode_dir)
    meta = json.loads((episode_dir / 'episode.json').read_text())
    streams = {s['name']: s for s in meta.get('streams', [])}
    recorded = set(meta.get('topics', {}))

    def topic_of(name, required=True):
        s = streams.get(name)
        if s is None or s['topic'] not in recorded:
            if required:
                raise SyncError(
                    f"{episode_dir.name}: stream '{name}' was not recorded "
                    f"(recorded: {sorted(recorded)})")
            return None
        return s['topic']

    notes = []
    state_topic = topic_of(spec.state_stream)
    action_name = spec.action_stream
    action_topic = topic_of(action_name, required=False)
    if action_topic is None:
        # Mock hardware: /control/joint_states is the state, recorded once.
        notes.append(f"'{action_name}' not recorded; action is "
                     f"'{spec.state_stream}' (mock hardware, where the "
                     'command is the state)')
        action_name, action_topic = spec.state_stream, state_topic

    cameras = spec.cameras or [n for n, s in streams.items()
                               if s.get('role') == 'camera'
                               and s['topic'] in recorded]
    if not cameras:
        raise SyncError(f'{episode_dir.name}: no camera stream was recorded')
    topics = {spec.state_stream: state_topic, action_name: action_topic}
    topics.update({c: topic_of(c) for c in cameras})
    mode_topic = topic_of(spec.mode_stream, required=spec.trim == 'teleop')
    if mode_topic:
        topics[spec.mode_stream] = mode_topic

    raw = read_streams(episode_dir, topics)

    # ---- joints, by name ------------------------------------------------
    state_msgs = raw[spec.state_stream]
    if not state_msgs:
        raise SyncError(f"{episode_dir.name}: '{spec.state_stream}' is empty")
    # The layout is the ROBOT MODEL's -- arm group then gripper group -- not
    # whatever order a driver happens to publish in.
    names = [j for g in spec.groups for j in g['joints']]
    for stream in {spec.state_stream, action_name}:
        seen = set(raw[stream][0][2].name) if raw[stream] else set()
        missing = [n for n in names if n not in seen]
        if missing:
            raise SyncError(
                f"{episode_dir.name}: '{stream}' does not carry {missing}, which "
                f'the robot model puts in the dataset (it carries {sorted(seen)}). '
                'Is --robot-config-package the robot this episode was recorded on?')

    def joint_series(stream):
        t, q = [], []
        for stamp, _, msg in raw[stream]:
            pos = positions_by_name(msg, names)
            if pos is not None and np.isfinite(stamp):
                t.append(stamp)
                q.append(pos)
        if not t:
            raise SyncError(f"{episode_dir.name}: '{stream}' never carried "
                            f'all of {names}')
        return Series.from_samples(t, q)

    state_series, state_dups = joint_series(spec.state_stream)
    if action_name == spec.state_stream:
        action_series, action_dups = state_series, 0
    else:
        action_series, action_dups = joint_series(action_name)

    # ---- camera timeline ------------------------------------------------
    primary = cameras[0]
    frames = sorted(raw[primary], key=lambda f: f[0])
    cam_t = np.array([f[0] for f in frames])
    cam_recv = np.array([f[1] for f in frames])
    true_rate = camera_rate(cam_t)
    fps = choose_fps(true_rate)

    span = None
    if spec.trim == 'teleop':
        mode = raw[spec.mode_stream]
        span = active_span([m[0] for m in mode], [m[2].mode for m in mode])
        if span is None:
            raise SyncError(f'{episode_dir.name}: the operator never engaged')
        keep = (cam_t >= span[0]) & (cam_t <= span[1])
    else:
        keep = np.ones(len(cam_t), dtype=bool)
    idx = np.flatnonzero(keep)
    if len(idx) < 2:
        raise SyncError(f'{episode_dir.name}: fewer than two camera frames '
                        'inside the teleop span')

    gaps = frame_gaps(cam_t[idx], fps)
    if gaps:
        worst = max(g[1] for g in gaps)
        raise SyncError(
            f'{episode_dir.name}: the camera skipped {len(gaps)} time(s), up to '
            f'{worst} frame(s) at once. The dataset assumes one frame every '
            f'1/{fps} s, so a hole would compress time around it.')

    query = cam_t[idx] + spec.camera_offset
    state, state_ok = sample(state_series, query, spec.max_joint_gap)
    action, action_ok = sample(action_series, query, spec.max_joint_gap)
    bad = np.flatnonzero(~(state_ok & action_ok))
    if len(bad):
        raise SyncError(
            f'{episode_dir.name}: {len(bad)} frame(s) have no joint sample '
            f'within {spec.max_joint_gap * 1000:.0f} ms (first at '
            f'{query[bad[0]] - cam_t[0]:.2f} s). The joint stream dropped out.')

    jpegs = {primary: [bytes(frames[i][2].data) for i in idx]}
    for cam in cameras[1:]:
        # Other cameras: the nearest capture within half a frame period.
        other = sorted(raw[cam], key=lambda f: f[0])
        ot = np.array([f[0] for f in other])
        j = np.clip(np.searchsorted(ot, cam_t[idx]), 1, len(ot) - 1)
        j = np.where(np.abs(ot[j - 1] - cam_t[idx]) < np.abs(ot[j] - cam_t[idx]),
                     j - 1, j)
        miss = np.abs(ot[j] - cam_t[idx]) > 0.5 / fps
        if miss.any():
            raise SyncError(f"{episode_dir.name}: camera '{cam}' has no frame "
                            f'within half a period of {int(miss.sum())} '
                            f"'{primary}' frame(s)")
        jpegs[cam] = [bytes(other[k][2].data) for k in j]

    report = {
        'camera_rate_hz': round(true_rate, 4),
        'dataset_fps': fps,
        'wall_drift_ms': round(1000 * (len(idx) / fps - (cam_t[idx[-1]]
                                                         - cam_t[idx[0]]
                                                         + 1.0 / true_rate)), 1),
        'camera_latency_ms': round(float(np.median(cam_recv - cam_t)) * 1000, 1),
        'frames_recorded': int(len(cam_t)),
        'frames_kept': int(len(idx)),
        'trim': ({'start_s': round(span[0] - cam_t[0], 2),
                  'end_s': round(span[1] - cam_t[0], 2),
                  'recorded_s': round(cam_t[-1] - cam_t[0], 2)}
                 if span else None),
        'repeated_joint_stamps_dropped': {spec.state_stream: state_dups,
                                          action_name: action_dups},
        'max_joint_bracket_ms': round(1000 * float(np.max(
            _bracket(state_series.t, query))), 2),
        'camera_offset_ms': round(spec.camera_offset * 1000, 1),
        'notes': notes,
    }
    arm = next(g for g in spec.groups if g['name'] == 'arm')
    report['motion_check'] = _motion_check(
        jpegs[primary], cam_t[idx], state_series,
        [names.index(j) for j in arm['joints']])

    return Bundle(
        source=episode_dir.name,
        task=spec.task or meta.get('task') or 'unknown',
        fps=fps,
        state_names=names,
        action_names=names,
        cameras=cameras,
        groups=spec.groups,
        capture_t=cam_t[idx],
        state=state.astype(np.float32),
        action=action.astype(np.float32),
        jpegs=jpegs,
        report=report)


def _bracket(t, query):
    i = np.clip(np.searchsorted(t, query), 1, len(t) - 1)
    return t[i] - t[i - 1]


def _motion_check(jpegs, frame_t, series: Series, arm_columns) -> dict:
    """Cross-correlate image motion with arm-joint speed; see sync.lag_verdict.

    The raw curve rides along under ``curve`` so `convert` can pool episodes;
    it is dropped before anything is written.
    """
    import cv2

    small = []
    for data in jpegs:
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
        small.append(cv2.resize(img, (160, 90),
                                interpolation=cv2.INTER_AREA).astype(np.float32))
    motion = np.array([np.mean(np.abs(small[k] - small[k - 1]))
                       for k in range(1, len(small))])
    lags, r, n = motion_correlation(frame_t, motion,
                                    Series(series.t, series.v[:, arm_columns]))
    period = float(np.median(np.diff(frame_t)))
    result = lag_verdict(lags, r, n, period)
    result['curve'] = (lags, r, n)
    return result


# ---- the bundle -----------------------------------------------------------

def write_bundle(bundle: Bundle, out_dir) -> Path:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    np.savez(out / 'arrays.npz', capture_t=bundle.capture_t,
             state=bundle.state, action=bundle.action)
    for cam, frames in bundle.jpegs.items():
        d = out / cam
        d.mkdir(exist_ok=True)
        for k, data in enumerate(frames):
            (d / f'{k:06d}.jpg').write_bytes(data)
    (out / 'meta.json').write_text(json.dumps({
        'source': bundle.source, 'task': bundle.task, 'fps': bundle.fps,
        'frames': len(bundle), 'state_names': bundle.state_names,
        'action_names': bundle.action_names, 'cameras': bundle.cameras,
        'groups': bundle.groups,
        'report': bundle.report}, indent=2))
    return out
