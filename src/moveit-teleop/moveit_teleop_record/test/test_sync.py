"""Time alignment for export. Every rule here was a real failure in real data."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

from moveit_teleop_record.extract import positions_by_name
from moveit_teleop_record.sync import (
    Series, SyncError, active_span, camera_rate, choose_fps, frame_gaps,
    lag_verdict, motion_correlation, pool_curves, sample,
)


# ---- repeated stamps ------------------------------------------------------
# The Piper driver republishes a joint sample when no new CAN frame arrived,
# repeating its header stamp: 51 of 6752 samples in a real episode. The cross-
# correlation came out NaN until they were removed.

def test_repeated_stamps_are_dropped_keeping_the_last():
    s, dropped = Series.from_samples([0.0, 0.005, 0.005, 0.010],
                                     [[1.0], [2.0], [9.0], [3.0]])
    assert dropped == 1
    assert list(s.t) == [0.0, 0.005, 0.010]
    assert s.v[1, 0] == 9.0


def test_out_of_order_samples_are_sorted():
    s, _ = Series.from_samples([0.2, 0.0, 0.1], [[2.0], [0.0], [1.0]])
    assert list(s.t) == [0.0, 0.1, 0.2]
    assert list(s.v[:, 0]) == [0.0, 1.0, 2.0]


def test_interpolation_never_divides_by_zero_after_dedup():
    s, _ = Series.from_samples([0.0, 0.0, 0.01], [[0.0], [0.0], [1.0]])
    values, ok = sample(s, [0.005], max_gap=0.05)
    assert ok.all() and np.isfinite(values).all()
    assert values[0, 0] == pytest.approx(0.5)


# ---- sampling -------------------------------------------------------------

def test_linear_interpolation_between_samples():
    s, _ = Series.from_samples([0.0, 0.01], [[0.0, 10.0], [1.0, 20.0]])
    values, ok = sample(s, [0.0025, 0.01], max_gap=0.05)
    assert ok.all()
    assert values[0] == pytest.approx([0.25, 12.5])
    assert values[1] == pytest.approx([1.0, 20.0])     # exactly on the last sample


def test_a_query_outside_the_series_is_not_ok():
    s, _ = Series.from_samples([1.0, 2.0], [[0.0], [1.0]])
    _, ok = sample(s, [0.5, 2.5], max_gap=10.0)
    assert not ok.any()


def test_a_query_across_a_dropout_is_not_ok():
    # A held value across a hole teaches a policy that the arm did not move.
    s, _ = Series.from_samples([0.0, 0.005, 0.300, 0.305], [[0], [0], [1], [1]])
    _, ok = sample(s, [0.0025, 0.150, 0.3025], max_gap=0.05)
    assert list(ok) == [True, False, True]


# ---- trimming -------------------------------------------------------------
# The recorded episode ended with a HOME: the planner moved the arm, so the
# recorded action there is not the operator's.

def test_active_span_is_first_to_last_teleop_status():
    TELEOP, ALIGNED, PLANNING = 2, 1, 3
    t = [0.0, 3.9, 10.0, 23.3, 24.0, 28.8, 29.4, 32.4]
    m = [ALIGNED, TELEOP, TELEOP, ALIGNED, TELEOP, TELEOP, PLANNING, ALIGNED]
    assert active_span(t, m) == (3.9, 28.8)


def test_an_episode_never_engaged_has_no_span():
    assert active_span([0.0, 1.0], [1, 1]) is None


# ---- the camera timeline --------------------------------------------------

def test_camera_rate_from_the_median_interval():
    stamps = np.arange(100) * 0.0666
    assert camera_rate(stamps) == pytest.approx(15.015, abs=0.01)


def test_fps_is_the_nearest_integer_when_close():
    # The 335L asked for 15 fps runs at 15.016.
    assert choose_fps(15.016) == 15


def test_a_camera_far_from_a_whole_rate_is_refused():
    # LeRobot timestamps are index/fps; 14.3 Hz exported as 14 fps drifts 2%.
    with pytest.raises(SyncError, match='whole frame rate'):
        choose_fps(14.3)


def test_frame_gaps_report_where_and_how_many():
    stamps = [0.0, 1 / 15, 2 / 15, 5 / 15, 6 / 15]
    assert frame_gaps(stamps, 15) == [(3, 2)]


def test_a_regular_camera_has_no_gaps():
    assert frame_gaps(np.arange(50) / 15.0, 15) == []


# ---- the measured cross-check ---------------------------------------------

def _synthetic(true_lag):
    """Image motion driven by joint speed, with the joint clock `true_lag` late."""
    rng = np.random.default_rng(0)
    t = np.arange(0.0, 20.0, 0.005)
    speed_profile = np.abs(np.sin(t * 1.3)) + 0.5 * np.abs(np.sin(t * 3.7 + 1.0))
    q = np.cumsum(speed_profile) * 0.005
    joints, _ = Series.from_samples(t + true_lag, q[:, None])
    frames = np.arange(0.5, 19.5, 1 / 15)
    motion = np.array([np.interp(0.5 * (a + b), t, speed_profile)
                       for a, b in zip(frames[:-1], frames[1:])])
    return frames, motion + rng.normal(0, 0.01, motion.shape), joints


@pytest.mark.parametrize('true_lag', [0.0, 0.030, -0.020])
def test_the_correlation_peak_recovers_a_known_offset(true_lag):
    frames, motion, joints = _synthetic(true_lag)
    lags, r, _ = motion_correlation(frames, motion, joints)
    peak = int(np.nanargmax(r))
    assert r[peak] > 0.8
    assert lags[peak] == pytest.approx(true_lag, abs=0.006)


# ---- joints by name -------------------------------------------------------
# Measured state lists joint1..joint6, gripper; the command stream lists
# gripper, joint1..joint6. By index, action[0] would be the gripper.

class _Msg:
    def __init__(self, name, position):
        self.name, self.position = name, position


def test_positions_are_taken_by_name_not_index():
    names = ['joint1', 'joint2', 'gripper']
    state = _Msg(['joint1', 'joint2', 'gripper'], [0.1, 0.2, 0.05])
    command = _Msg(['gripper', 'joint1', 'joint2'], [0.05, 0.1, 0.2])
    assert positions_by_name(state, names) == positions_by_name(command, names)


def test_a_message_missing_a_joint_is_rejected():
    assert positions_by_name(_Msg(['joint1'], [0.0]), ['joint1', 'gripper']) is None


# ---- GR00T modality.json --------------------------------------------------

def _writer():
    path = Path(__file__).resolve().parents[1] / 'moveit_teleop_record' / 'lerobot_writer.py'
    spec = importlib.util.spec_from_file_location('lerobot_writer', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)           # imports no lerobot at module level
    return module


PIPER = [{'name': 'arm', 'srdf_group': 'arm',
          'joints': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
          'units': ['rad'] * 6},
         {'name': 'gripper', 'srdf_group': 'gripper', 'joints': ['gripper'], 'units': ['m']}]
# The case the old name-guessing got wrong: no joint here contains "gripper".
FR3 = [{'name': 'arm', 'srdf_group': 'fr3_arm',
        'joints': [f'fr3_joint{i}' for i in range(1, 8)], 'units': ['rad'] * 7},
       {'name': 'gripper', 'srdf_group': 'fr3_hand',
        'joints': ['fr3_finger_joint1'], 'units': ['m']}]


def test_modality_matches_the_shape_of_gr00ts_own_demo():
    m = _writer().build_modality(PIPER, ['gripper_camera'])
    assert m['state'] == {'arm': {'start': 0, 'end': 6}, 'gripper': {'start': 6, 'end': 7}}
    assert m['action'] == m['state']
    assert m['video'] == {'gripper_camera': {
        'original_key': 'observation.images.gripper_camera'}}
    assert m['annotation'] == {'human.task_description': {'original_key': 'task_index'}}


def test_modality_follows_the_robot_model_not_joint_names():
    m = _writer().build_modality(FR3, ['wrist'])
    assert m['state'] == {'arm': {'start': 0, 'end': 7}, 'gripper': {'start': 7, 'end': 8}}


def test_an_arm_without_a_gripper_has_one_group():
    m = _writer().build_modality(PIPER[:1], ['cam'])
    assert m['state'] == {'arm': {'start': 0, 'end': 6}}


def _export(groups, **gr00t):
    settings = {'action_horizon': 16, 'arm_action': 'relative', 'gripper_action': 'absolute'}
    settings.update(gr00t)
    return {'robot': {'robot_name': 'r', 'robot_type': 'r', 'config': '/c/teleop.yaml',
                      'groups': groups},
            'dataset': {'gr00t': settings}}


def test_generated_gr00t_config_is_valid_python_naming_the_groups():
    import ast
    source = _writer().build_gr00t_config(_export(FR3), ['wrist'], fps=15)
    tree = ast.parse(source)
    assert "modality_keys=['arm', 'gripper']" in source
    assert "modality_keys=['wrist']" in source
    assert 'delta_indices=list(range(16))' in source
    assert source.count('ActionRepresentation.RELATIVE') == 1
    assert source.count('ActionRepresentation.ABSOLUTE') == 1
    assert 'fr3_finger_joint1' in source           # layout documented in the file
    assert any(isinstance(n, ast.Call) and getattr(n.func, 'id', '') == 'register_modality_config'
               for n in ast.walk(tree))


def test_gr00t_action_representation_comes_from_the_settings():
    source = _writer().build_gr00t_config(
        _export(PIPER, arm_action='absolute', action_horizon=8), ['cam'], fps=15)
    assert source.count('ActionRepresentation.ABSOLUTE') == 2
    assert 'delta_indices=list(range(8))' in source


def test_the_writer_script_imports_nothing_from_ros():
    # It runs in the LeRobot interpreter, which cannot import ROS or this
    # package. Checked on the real imports, not on text: the dataset README it
    # writes legitimately mentions the ros2 command.
    import ast
    tree = ast.parse((Path(__file__).resolve().parents[1] / 'moveit_teleop_record'
                      / 'lerobot_writer.py').read_text())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported |= {a.name.split('.')[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom):
            assert node.level == 0, 'relative import in a script run by path'
            imported.add((node.module or '').split('.')[0])
    forbidden = {'rclpy', 'rosbag2_py', 'rosidl_runtime_py', 'moveit_teleop_record',
                 'moveit_teleop_core', 'ament_index_python'}
    assert not imported & forbidden, sorted(imported & forbidden)


# ---- judging the correlation curve ----------------------------------------
# A real 4-episode session: one short episode peaked at -100 ms while scoring
# only 0.02 lower at 0 ms. Reading the peak alone called it misaligned; the
# verdict now accounts for how flat the curve is.



def _flat_curve(peak_at, peak_r=0.56, drop_at_zero=0.02, width=0.12):
    lags = np.arange(-0.200, 0.2001, 0.002)
    r = peak_r - (drop_at_zero / max(abs(peak_at), 1e-6) ** 2) * (lags - peak_at) ** 2
    return lags, r


def test_a_flat_curve_whose_peak_barely_beats_zero_is_aligned():
    lags, r = _flat_curve(peak_at=-0.100)
    v = lag_verdict(lags, r, n=133, frame_period=1 / 15)
    assert v['offset_ms'] == pytest.approx(-100, abs=2)
    assert v['verdict'] == 'aligned'
    assert v['band_ms'][0] <= 0 <= v['band_ms'][1]


def test_a_sharp_peak_far_from_zero_is_misaligned():
    lags = np.arange(-0.200, 0.2001, 0.002)
    r = 0.9 - 40.0 * (lags - 0.150) ** 2           # sharp, well away from zero
    v = lag_verdict(lags, r, n=2000, frame_period=1 / 15)
    assert v['verdict'] == 'MISALIGNED'


def test_weak_correlation_is_inconclusive_not_a_finding():
    lags, r = _flat_curve(peak_at=0.150, peak_r=0.2)
    assert lag_verdict(lags, r, n=500, frame_period=1 / 15)['verdict'].startswith('inconclusive')


def test_pooling_weights_episodes_by_their_length():
    lags = np.array([-0.1, 0.0, 0.1])
    lags_, r, n = pool_curves([(lags, np.array([0.1, 0.5, 0.2]), 300),
                               (lags, np.array([0.9, 0.1, 0.1]), 100)])
    assert n == 400
    assert r == pytest.approx([(0.1 * 300 + 0.9 * 100) / 400,
                               (0.5 * 300 + 0.1 * 100) / 400,
                               (0.2 * 300 + 0.1 * 100) / 400])


@pytest.mark.parametrize('true_lag', [0.0, 0.150])
def test_the_verdict_on_a_synthetic_session(true_lag):
    frames, motion, joints = _synthetic(true_lag)
    lags, r, n = motion_correlation(frames, motion, joints)
    v = lag_verdict(lags, r, n, frame_period=1 / 15)
    assert v['verdict'] == ('aligned' if true_lag == 0.0 else 'MISALIGNED')
