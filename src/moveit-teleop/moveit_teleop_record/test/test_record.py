"""Tests for recording config and dataset export."""

import array
from pathlib import Path

import numpy as np
import pytest

from moveit_teleop_record.config import (
    ExportOptions, RecordingConfig, RecordingConfigError, Stream,
)
from moveit_teleop_record.exporters import flatten_message

# ---- config ---------------------------------------------------------------

# ---- topic placeholders ---------------------------------------------------
# Some topics depend on the SOLVER and DEVICE, not on the robot. Hardcoding
# them made recording.yaml correct for servo+fake and silently unusable on
# every other combination: the required action stream did not exist, so the
# recorder refused every episode.


def test_placeholder_is_expanded():
    cfg = RecordingConfig.from_dict(
        {'streams': [{'topic': '${target_topic}', 'role': 'action',
                      'name': 'ee_target'}]},
        {'target_topic': '/ik_node/pose_target_cmds'})
    assert cfg.streams[0].topic == '/ik_node/pose_target_cmds'


def test_derived_name_uses_the_expanded_topic():
    cfg = RecordingConfig.from_dict(
        {'streams': [{'topic': '${target_topic}', 'role': 'action'}]},
        {'target_topic': '/ik_node/pose_target_cmds'})
    assert cfg.streams[0].name == 'ik_node_pose_target_cmds'


def test_unknown_placeholder_is_an_error_not_a_literal_topic():
    # A literal '${target_topic}' would be subscribed to, found absent, and
    # reported as a missing stream -- sending you to look at the robot.
    with pytest.raises(RecordingConfigError, match='unknown placeholders'):
        RecordingConfig.from_dict(
            {'streams': [{'topic': '${typo}', 'role': 'action'}]},
            {'target_topic': '/x'})


def test_topics_without_placeholders_are_untouched():
    cfg = RecordingConfig.from_dict(
        {'streams': [{'topic': '/control/joint_states',
                      'role': 'observation'}]},
        {'target_topic': '/x'})
    assert cfg.streams[0].topic == '/control/joint_states'


def test_substitutions_are_optional():
    # Every existing caller passes none.
    cfg = RecordingConfig.from_dict(
        {'streams': [{'topic': '/js', 'role': 'observation'}]})
    assert cfg.streams[0].topic == '/js'



def _cfg(**over):
    base = {
        'output_dir': '/tmp/eps',
        'streams': [
            {'topic': '/js', 'name': 'joint_states', 'role': 'observation',
             'required': True, 'fields': ['position']},
            {'topic': '/act', 'name': 'ee_target', 'role': 'action', 'required': True},
        ],
        'cameras': [{'topic': '/cam', 'name': 'wrist', 'resize': [224, 224]}],
        'sensors': [{'topic': '/ft', 'name': 'wrist_ft'}],
    }
    base.update(over)
    return RecordingConfig.from_dict(base)


def test_cameras_and_sensors_become_streams_with_their_roles():
    """Adding a camera must be config only -- no code path of its own."""
    c = _cfg()
    assert [s.name for s in c.by_role('camera')] == ['wrist']
    assert [s.name for s in c.by_role('sensor')] == ['wrist_ft']
    assert len(c.streams) == 4


def test_camera_extra_keys_are_preserved_for_the_exporter():
    c = _cfg()
    assert c.by_role('camera')[0].extra['resize'] == [224, 224]


def test_duplicate_stream_names_are_rejected():
    with pytest.raises(RecordingConfigError, match='duplicate'):
        _cfg(sensors=[{'topic': '/other', 'name': 'joint_states'}])


def test_unknown_role_is_rejected():
    with pytest.raises(RecordingConfigError, match='role'):
        RecordingConfig.from_dict(
            {'streams': [{'topic': '/x', 'name': 'x', 'role': 'nonsense'}]})


def test_empty_config_is_rejected():
    with pytest.raises(RecordingConfigError, match='nothing to record'):
        RecordingConfig.from_dict({})


def test_stream_name_defaults_from_topic():
    s = Stream.from_dict({'topic': '/a/b/c', 'role': 'context'})
    assert s.name == 'a_b_c'


def test_export_options_validated():
    with pytest.raises(RecordingConfigError):
        ExportOptions(default_interpolation='magic')
    with pytest.raises(RecordingConfigError):
        ExportOptions(rate_hz=0)


def test_metadata_round_trips_roles_and_fields():
    """The sidecar must freeze roles, or a dataset changes meaning when the
    config is later edited."""
    meta = _cfg().to_metadata()
    by_name = {s['name']: s for s in meta['streams']}
    assert by_name['joint_states']['role'] == 'observation'
    assert by_name['joint_states']['fields'] == ['position']
    assert by_name['wrist']['role'] == 'camera'


# ---- flattening -----------------------------------------------------------


class _Fake:
    """Stands in for a ROS message without needing a ROS environment."""

    def __init__(self, **kw):
        self._f = kw
        for k, v in kw.items():
            setattr(self, k, v)

    def get_fields_and_field_types(self):
        return {k: '' for k in self._f}


def test_array_array_is_flattened():
    """rclpy gives float64[] as array.array, not list.

    Missing this dropped every numeric array silently: the export produced a
    valid-looking dataset whose observation vector was empty.
    """
    msg = _Fake(position=array.array('d', [0.1, 0.2, 0.3]))
    assert flatten_message(msg) == {
        'position.0': 0.1, 'position.1': 0.2, 'position.2': 0.3}


def test_lists_tuples_and_ndarrays_all_flatten():
    for seq in ([1.0, 2.0], (1.0, 2.0), np.array([1.0, 2.0]),
                array.array('d', [1.0, 2.0])):
        assert flatten_message(_Fake(v=seq)) == {'v.0': 1.0, 'v.1': 2.0}


def test_strings_are_not_features():
    """JointState.name is labels; including it would poison the vector."""
    msg = _Fake(name=['joint1', 'joint2'], position=array.array('d', [0.5, 0.6]))
    assert flatten_message(msg) == {'position.0': 0.5, 'position.1': 0.6}


def test_bools_become_floats():
    assert flatten_message(_Fake(clutch=True, valid=False)) == {
        'clutch': 1.0, 'valid': 0.0}


def test_nested_messages_flatten_with_dotted_names():
    inner = _Fake(x=1.0, y=2.0)
    assert flatten_message(_Fake(position=inner)) == {
        'position.x': 1.0, 'position.y': 2.0}


def test_header_is_excluded():
    """Timestamps are carried by the frame, not as a feature."""
    msg = _Fake(header=_Fake(seq=3), value=1.0)
    assert flatten_message(msg) == {'value': 1.0}


def test_fields_filter_restricts_the_walk():
    msg = _Fake(position=array.array('d', [1.0]),
                velocity=array.array('d', [2.0]),
                effort=array.array('d', [3.0]))
    assert flatten_message(msg, ['position', 'velocity']) == {
        'position.0': 1.0, 'velocity.0': 2.0}


def test_missing_requested_field_is_skipped_not_fatal():
    assert flatten_message(_Fake(a=1.0), ['a', 'absent']) == {'a': 1.0}


# ---- one topic, two stream names ------------------------------------------
# joint_states is ${joint_states_topic}: the driver's measured state on
# hardware, but /control/joint_states on mock hardware -- the topic
# joint_commands names outright. A bag holds a topic once and every
# topic->stream map keeps one name, so keeping both would be a silent bug.

def _two_streams_one_topic(state_topic):
    return RecordingConfig.from_dict({
        'streams': [
            {'topic': '${joint_states_topic}', 'role': 'observation',
             'name': 'joint_states', 'required': True},
            {'topic': '/control/joint_states', 'role': 'action',
             'name': 'joint_commands', 'required': False},
        ]}, {'joint_states_topic': state_topic})


def test_hardware_records_state_and_command_separately():
    config = _two_streams_one_topic('/feedback/joint_states')
    assert [s.name for s in config.streams] == ['joint_states', 'joint_commands']
    assert config.shadowed == []


def test_mock_hardware_records_the_shared_topic_once_under_the_first_name():
    config = _two_streams_one_topic('/control/joint_states')
    assert [s.name for s in config.streams] == ['joint_states']
    assert config.shadowed == [
        ('joint_commands', '/control/joint_states', 'joint_states')]


def test_the_shipped_recording_config_resolves_on_both_setups():
    # The file as installed, not a hand-built dict: this is what the recorder
    # actually loads.
    path = (Path(__file__).resolve().parents[2]
            / 'moveit_teleop_bringup' / 'config' / 'recording.yaml')
    base = {'target_topic': '/ik_node/pose_target_cmds',
            'input_topic': '/xr_device/input',
            'solver_status_topic': '/ik_node/status'}

    hardware = RecordingConfig.from_file(
        path, {**base, 'joint_states_topic': '/feedback/joint_states'})
    by_name = {s.name: s for s in hardware.streams}
    assert by_name['joint_states'].topic == '/feedback/joint_states'
    assert by_name['joint_states'].role == 'observation'
    assert by_name['joint_commands'].topic == '/control/joint_states'
    assert by_name['joint_commands'].role == 'action'
    assert by_name['gripper_camera'].role == 'camera'
    assert by_name['gripper_camera'].topic.endswith('/compressed')
    assert not by_name['gripper_camera'].required

    sim = RecordingConfig.from_file(
        path, {**base, 'joint_states_topic': '/control/joint_states'})
    assert 'joint_commands' not in {s.name for s in sim.streams}
