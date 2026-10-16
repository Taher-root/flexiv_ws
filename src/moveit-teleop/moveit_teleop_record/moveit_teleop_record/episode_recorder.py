"""Record teleoperation episodes as MCAP bags with clean boundaries.

Segmentation is driven by the mode manager rather than by a human remembering
to start and stop a recorder, because for imitation learning the boundaries ARE
the data: a demonstration that begins before the operator engaged, or runs on
through a watchdog halt, is a corrupted sample that will be trained on.

Each episode is one bag directory plus a JSON sidecar. The sidecar carries what
a bag cannot: the task label, whether the episode ended cleanly or was faulted
out, and a timeline of mode transitions. A converter reads both.

Deliberately NOT here: any LeRobot or RLDS schema. Conversion is offline
(``moveit_teleop_record.convert``) so the training format never becomes a
runtime dependency of teleoperation.
"""

from __future__ import annotations

import json
import shutil
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from rclpy.serialization import serialize_message

import rosbag2_py
from moveit_teleop_msgs.msg import TeleopEvent, TeleopStatus

from .config import RecordingConfig

EVENT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST, depth=10,
    durability=DurabilityPolicy.VOLATILE)


class EpisodeRecorder(Node):

    def __init__(self):
        super().__init__('episode_recorder')

        # Everything about WHAT to record lives in the config file. Adding a
        # camera or a sensor is an entry there, never a change here.
        self.declare_parameter('config_file', '')
        self.declare_parameter('event_topic', '/fake_device/event')
        self.declare_parameter('status_topic', '/mode_manager/status')
        self.declare_parameter('task', '')
        self.declare_parameter('output_dir', '')   # optional override
        # Topics that depend on the SESSION rather than the robot -- which
        # solver is running, which device is driving. Passed as 'name=value'
        # so the config file can refer to them as ${name} and stay correct
        # on every solver/device combination.
        self.declare_parameter('topic_substitutions', [''])

        config_file = self.get_parameter('config_file').value
        if not config_file:
            raise RuntimeError(
                'config_file is required: the recorder is configured by a '
                'recording.yaml, not by node parameters')
        substitutions = {}
        for entry in self.get_parameter('topic_substitutions').value or []:
            if not entry:
                continue
            if '=' not in entry:
                raise ValueError(
                    f'topic_substitutions entry {entry!r} is not name=value')
            name, _, value = entry.partition('=')
            substitutions[name.strip()] = value.strip()
        self.config = RecordingConfig.from_file(config_file, substitutions)

        override = self.get_parameter('output_dir').value
        self.output_dir = (Path(override).expanduser() if override
                           else self.config.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.storage_id = self.config.storage_id
        self.discard_on_halt = self.config.discard_on_halt
        self.topics = [s.topic for s in self.config.streams]
        self._streams_by_topic = self.config.by_topic()

        self._writer = None
        self._path = None
        self._meta = {}
        self._subs = []
        self._known_types = {}
        self._last_mode = None

        self.create_subscription(
            TeleopEvent, self.get_parameter('event_topic').value,
            self._on_event, EVENT_QOS)
        self.create_subscription(
            TeleopStatus, self.get_parameter('status_topic').value,
            self._on_status, 10)

        roles = {}
        for stream in self.config.streams:
            roles.setdefault(stream.role, []).append(stream.name)
        self.get_logger().info(
            f'recording to {self.output_dir} ({self.storage_id}) from '
            f'{config_file}')
        for role, names in sorted(roles.items()):
            self.get_logger().info(f'  {role}: {", ".join(names)}')
        for dropped, topic, kept in self.config.shadowed:
            # Expected on mock hardware, where the command echo IS the state.
            self.get_logger().info(
                f"  '{dropped}' shares {topic} with '{kept}'; recorded once, "
                f"as '{kept}'")
        self.get_logger().info('waiting for RECORD_START')

    # ---- episode lifecycle ------------------------------------------------

    def _resolve_types(self) -> dict:
        """Map topic -> type name from what is actually being published.

        Resolved at episode start rather than configured, so adding a camera
        to the topic list needs no schema declaration.
        """
        available = dict(self.get_topic_names_and_types())
        resolved, missing_required, missing_optional = {}, [], []
        for stream in self.config.streams:
            types = available.get(stream.topic)
            if types:
                resolved[stream.topic] = types[0]
            elif stream.required:
                missing_required.append(stream.topic)
            else:
                missing_optional.append(stream.topic)

        if missing_optional:
            # A sensor that is not plugged in is normal; say so and carry on.
            self.get_logger().info(
                f'optional streams absent, not recorded: {missing_optional}')
            cameras = [s.name for s in self.config.streams
                       if s.role == 'camera' and s.topic in missing_optional]
            if cameras:
                # But a camera is almost always the point of the episode, and
                # forgetting to start it produces a take with no images that
                # still LOOKS successful. Loud, not fatal: simulation and the
                # fake device legitimately have no camera.
                self.get_logger().warn(
                    f'recording WITHOUT camera(s) {cameras}: nothing is '
                    'publishing on their topics. Start the camera if this '
                    'episode is meant to have images.')
        if missing_required:
            # A missing REQUIRED stream produces an episode that looks fine and
            # is unusable for training. Refuse rather than record a hole.
            raise RuntimeError(
                f'required streams are not being published: {missing_required}')
        return resolved

    def _start(self, episode_id: str, task: str) -> None:
        if self._writer is not None:
            self.get_logger().warn('already recording; ignoring RECORD_START')
            return

        try:
            known = self._resolve_types()
        except RuntimeError as exc:
            self.get_logger().error(f'refusing to start an episode: {exc}')
            return

        stamp = time.strftime('%Y%m%d_%H%M%S')
        name = f'{stamp}_{episode_id}' if episode_id else stamp
        self._path = self.output_dir / name
        if self._path.exists():
            shutil.rmtree(self._path)

        self._writer = rosbag2_py.SequentialWriter()
        self._writer.open(
            rosbag2_py.StorageOptions(uri=str(self._path), storage_id=self.storage_id),
            rosbag2_py.ConverterOptions('', ''))

        self._known_types = known
        for topic, type_name in self._known_types.items():
            self._writer.create_topic(rosbag2_py.TopicMetadata(
                id=0, name=topic, type=type_name, serialization_format='cdr'))

        self._meta = {
            'episode_id': episode_id,
            'task': task or self.get_parameter('task').value or 'unknown',
            'started_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'topics': self._known_types,
            'missing_topics': [t for t in self.topics if t not in self._known_types],
            'mode_timeline': [],
            'halted_during_episode': False,
            # The stream roles are frozen into the episode so it stays
            # interpretable even if the config is edited later. Without this a
            # dataset silently changes meaning when someone renames a stream.
            **self.config.to_metadata(),
        }
        self._subscribe_all()
        self.get_logger().info(f'episode started -> {self._path}')

    def _subscribe_all(self) -> None:
        from rosidl_runtime_py.utilities import get_message
        for topic, type_name in self._known_types.items():
            msg_type = get_message(type_name)
            # Generic subscriptions use a wide-open QoS so that a BEST_EFFORT
            # publisher (the device stream) is still captured; a RELIABLE
            # subscriber would silently receive nothing from it.
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=50,
                durability=DurabilityPolicy.VOLATILE)
            self._subs.append(self.create_subscription(
                msg_type, topic,
                lambda msg, t=topic: self._write(t, msg), qos))

    def _write(self, topic: str, msg) -> None:
        if self._writer is None:
            return
        self._writer.write(topic, serialize_message(msg),
                           self.get_clock().now().nanoseconds)

    def _stop(self, discard: bool, reason: str) -> None:
        if self._writer is None:
            return
        for sub in self._subs:
            self.destroy_subscription(sub)
        self._subs = []
        del self._writer          # closes and finalises the bag
        self._writer = None

        halted = self._meta.get('halted_during_episode', False)
        if halted and self.discard_on_halt and not discard:
            discard = True
            reason = 'halted during episode (discard_on_halt)'

        if discard:
            shutil.rmtree(self._path, ignore_errors=True)
            self.get_logger().info(f'episode discarded ({reason}): {self._path.name}')
        else:
            self._meta['ended_utc'] = time.strftime(
                '%Y-%m-%dT%H:%M:%SZ', time.gmtime())
            self._meta['end_reason'] = reason
            (self._path / 'episode.json').write_text(
                json.dumps(self._meta, indent=2))
            self.get_logger().info(f'episode saved ({reason}): {self._path.name}')
        self._path = None
        self._meta = {}

    # ---- inputs -----------------------------------------------------------

    def _on_event(self, msg: TeleopEvent) -> None:
        if msg.type == TeleopEvent.RECORD_START:
            # The payload labels this TAKE, not the task. Letting it become the
            # task gave every episode a task of its own, which makes a
            # multi-episode dataset look like a set of one-shot tasks.
            self._start(msg.payload or f'ep{int(time.time())}', task='')
        elif msg.type in (TeleopEvent.RECORD_STOP, TeleopEvent.EPISODE_SAVE):
            self._stop(discard=False, reason='operator save')
        elif msg.type == TeleopEvent.EPISODE_DISCARD:
            if self._writer is not None:
                self._stop(discard=True, reason='operator discard')
            else:
                self._discard_last()

    def _discard_last(self) -> None:
        """Delete the most recently saved episode: the demonstrator's undo."""
        episodes = sorted((p for p in self.output_dir.iterdir() if p.is_dir()),
                          key=lambda p: p.stat().st_mtime)
        if not episodes:
            self.get_logger().warn('EPISODE_DISCARD with nothing to discard')
            return
        victim = episodes[-1]
        shutil.rmtree(victim, ignore_errors=True)
        self.get_logger().info(f'discarded last saved episode: {victim.name}')

    def _on_status(self, msg: TeleopStatus) -> None:
        if self._writer is None:
            self._last_mode = msg.mode
            return
        if msg.mode != self._last_mode:
            self._meta['mode_timeline'].append({
                'mode': int(msg.mode),
                'utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                'message': msg.message,
            })
            if msg.mode == TeleopStatus.HALTED:
                self._meta['halted_during_episode'] = True
                self.get_logger().warn(
                    'halt during episode: this take is marked and will be '
                    'discarded unless discard_on_halt is false')
            self._last_mode = msg.mode

    def destroy_node(self) -> bool:
        # A bag left open on shutdown is a corrupt bag.
        if self._writer is not None:
            self._stop(discard=False, reason='recorder shutdown')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = EpisodeRecorder()
    except Exception as exc:  # noqa: BLE001
        rclpy.logging.get_logger('episode_recorder').fatal(str(exc))
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
