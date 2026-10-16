"""Parse the recording config: which streams exist and what each one means.

Topic names may contain ``${placeholder}`` references, because some of them
are not properties of the ROBOT at all -- they depend on which solver and
which input device the stack was launched with. Hardcoding them made the config
correct for exactly one combination (servo + fake device) and silently wrong
for every other: on the IK path the action stream simply does not exist, and
the recorder refuses every episode.

The launch file knows the answers, so it passes them in as substitutions and
this file stays a declaration of what a training sample CONTAINS.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

import yaml


_PLACEHOLDER = re.compile(r'\$\{([a-z_0-9]+)\}')


def expand(text: str, substitutions: dict) -> str:
    """Replace every ``${name}`` in *text*.

    An unknown placeholder is an error, never a silent pass-through: a
    literal ``${target_topic}`` would be subscribed to as a topic name,
    found absent, and reported as a missing stream -- which sends you
    looking at the robot instead of at the typo.
    """
    missing = [m.group(1) for m in _PLACEHOLDER.finditer(text)
               if m.group(1) not in substitutions]
    if missing:
        raise RecordingConfigError(
            f'{text!r} references unknown placeholders {sorted(set(missing))}; '
            f'known: {sorted(substitutions)}')
    return _PLACEHOLDER.sub(lambda m: substitutions[m.group(1)], text)


class RecordingConfigError(ValueError):
    pass


@dataclass
class Stream:
    """One recorded topic, plus what it means to a training sample."""

    topic: str
    name: str
    role: str                      # observation | action | context | camera | sensor
    required: bool = False
    fields: list = field(default_factory=list)
    interpolation: str = ''        # '' -> use the export default
    extra: dict = field(default_factory=dict)

    ROLES = ('observation', 'action', 'context', 'camera', 'sensor')

    @classmethod
    def from_dict(cls, raw: dict, default_role: str = '',
                  substitutions: dict = None) -> 'Stream':
        if 'topic' not in raw:
            raise RecordingConfigError(f'stream entry has no topic: {raw}')
        topic = expand(raw['topic'], substitutions or {})
        role = raw.get('role', default_role)
        if role not in cls.ROLES:
            raise RecordingConfigError(
                f"stream {raw['topic']!r} has role {role!r}; "
                f'expected one of {cls.ROLES}')
        known = {'topic', 'name', 'role', 'required', 'fields', 'interpolation'}
        return cls(
            topic=topic,
            name=raw.get('name') or topic.strip('/').replace('/', '_'),
            role=role,
            required=bool(raw.get('required', False)),
            fields=list(raw.get('fields', []) or []),
            interpolation=raw.get('interpolation', ''),
            extra={k: v for k, v in raw.items() if k not in known},
        )


@dataclass
class ExportOptions:
    rate_hz: float = 30.0
    max_staleness: float = 0.05
    default_interpolation: str = 'zero_order_hold'

    INTERPOLATIONS = ('zero_order_hold', 'linear')

    def __post_init__(self):
        if self.default_interpolation not in self.INTERPOLATIONS:
            raise RecordingConfigError(
                f'default_interpolation must be one of {self.INTERPOLATIONS}')
        if self.rate_hz <= 0:
            raise RecordingConfigError('rate_hz must be positive')


@dataclass
class RecordingConfig:
    output_dir: Path
    storage_id: str
    discard_on_halt: bool
    streams: list
    export: ExportOptions
    #: (dropped_name, topic, kept_name) for streams whose topic resolved to
    #: one an earlier stream already records. See from_dict.
    shadowed: list = field(default_factory=list)

    @classmethod
    def from_file(cls, path, substitutions: dict = None) -> 'RecordingConfig':
        raw = yaml.safe_load(Path(path).expanduser().read_text())
        if 'recording' not in raw:
            raise RecordingConfigError(
                f'{path}: expected a top-level "recording:" key')
        return cls.from_dict(raw['recording'], substitutions)

    @classmethod
    def from_dict(cls, raw: dict,
                  substitutions: dict = None) -> 'RecordingConfig':
        # Cameras and sensors are separate lists purely so the config reads
        # clearly; downstream they are streams like any other, distinguished
        # by role.
        subs = substitutions or {}
        streams = [Stream.from_dict(s, '', subs)
                   for s in (raw.get('streams') or [])]
        streams += [Stream.from_dict(s, 'camera', subs)
                    for s in (raw.get('cameras') or [])]
        streams += [Stream.from_dict(s, 'sensor', subs)
                    for s in (raw.get('sensors') or [])]

        if not streams:
            raise RecordingConfigError('no streams configured; nothing to record')

        seen = {}
        for stream in streams:
            if stream.name in seen:
                raise RecordingConfigError(
                    f'duplicate stream name {stream.name!r} '
                    f'({seen[stream.name]} and {stream.topic})')
            seen[stream.name] = stream.topic

        # Two streams can legitimately resolve to ONE topic once placeholders
        # are expanded. The case that matters: `joint_states` is
        # ${joint_states_topic}, which is the driver's measured state on
        # hardware but /control/joint_states on mock hardware -- the very
        # topic `joint_commands` names. There the command IS the state, so
        # nothing is lost by recording it once.
        #
        # Keeping both would be a silent bug rather than a loud one: a bag
        # holds a topic once, and every topic->stream map downstream would keep
        # whichever name it saw last. So the FIRST declaration wins (list the
        # stream that must survive first) and the other is reported.
        kept, owner, shadowed = [], {}, []
        for stream in streams:
            if stream.topic in owner:
                shadowed.append((stream.name, stream.topic, owner[stream.topic]))
                continue
            owner[stream.topic] = stream.name
            kept.append(stream)
        streams = kept

        return cls(
            output_dir=Path(raw.get('output_dir', '~/teleop_episodes')).expanduser(),
            storage_id=raw.get('storage_id', 'mcap'),
            discard_on_halt=bool(raw.get('discard_on_halt', True)),
            streams=streams,
            export=ExportOptions(**(raw.get('export') or {})),
            shadowed=shadowed,
        )

    def by_role(self, role: str) -> list:
        return [s for s in self.streams if s.role == role]

    def by_topic(self) -> dict:
        return {s.topic: s for s in self.streams}

    def to_metadata(self) -> dict:
        """The shape written into an episode sidecar.

        The exporter reads this rather than the config file, so an episode
        stays interpretable even if the config is later edited -- otherwise a
        dataset silently changes meaning when someone renames a stream.
        """
        return {
            'streams': [
                {'topic': s.topic, 'name': s.name, 'role': s.role,
                 'fields': s.fields, 'interpolation': s.interpolation,
                 'extra': s.extra}
                for s in self.streams
            ],
            'export': {
                'rate_hz': self.export.rate_hz,
                'max_staleness': self.export.max_staleness,
                'default_interpolation': self.export.default_interpolation,
            },
        }
