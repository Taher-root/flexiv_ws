"""Tests for episode discovery and the playback timeline.

No ROS graph: everything about WHICH episode and WHERE IN IT lives in
library.py precisely so it can be tested like this.
"""

import json
import re
from pathlib import Path

import pytest

from moveit_teleop_replay.library import (
    Timeline, discover, mode_timeline,
)


def write_episode(root, name, started='2026-09-09T17:43:22Z', **extra):
    path = root / name
    path.mkdir()
    meta = {'episode_id': name, 'task': 'pick', 'started_utc': started,
            'end_reason': 'operator save', 'halted_during_episode': False}
    meta.update(extra)
    (path / 'episode.json').write_text(json.dumps(meta))
    return path


# ---- discovery ------------------------------------------------------------

def test_missing_directory_is_empty_not_an_error(tmp_path):
    assert discover(tmp_path / 'nope') == []


def test_newest_episode_comes_first(tmp_path):
    write_episode(tmp_path, 'a', started='2026-09-09T10:00:00Z')
    write_episode(tmp_path, 'c', started='2026-09-09T12:00:00Z')
    write_episode(tmp_path, 'b', started='2026-09-09T11:00:00Z')
    assert [e.name for e in discover(tmp_path)] == ['c', 'b', 'a']


def test_directory_without_a_sidecar_is_skipped(tmp_path):
    write_episode(tmp_path, 'good')
    (tmp_path / 'not_an_episode').mkdir()
    assert [e.name for e in discover(tmp_path)] == ['good']


def test_one_corrupt_sidecar_does_not_hide_the_others(tmp_path):
    # The episodes directory is also where partial and discarded episodes land.
    write_episode(tmp_path, 'good')
    bad = tmp_path / 'bad'
    bad.mkdir()
    (bad / 'episode.json').write_text('{ this is not json')
    assert [e.name for e in discover(tmp_path)] == ['good']


def test_summary_carries_what_a_list_needs(tmp_path):
    write_episode(tmp_path, 'ep1', task='fold', halted_during_episode=True)
    summary = discover(tmp_path)[0]
    assert summary.task == 'fold'
    assert summary.halted is True
    assert summary.end_reason == 'operator save'


# ---- mode timeline --------------------------------------------------------

def test_mode_timeline_is_relative_to_episode_start():
    meta = {'mode_timeline': [
        {'mode': 2, 'utc': '2026-09-09T17:43:24Z'},
        {'mode': 1, 'utc': '2026-09-09T17:43:31Z'},
    ]}
    times, values = mode_timeline(meta, '2026-09-09T17:43:22Z')
    assert times == [2.0, 9.0]
    assert values == [2, 1]


def test_unparseable_start_yields_no_timeline_rather_than_wrong_one():
    meta = {'mode_timeline': [{'mode': 2, 'utc': '2026-09-09T17:43:24Z'}]}
    assert mode_timeline(meta, 'not a timestamp') == ([], [])


def test_unparseable_entries_are_dropped_not_guessed():
    meta = {'mode_timeline': [
        {'mode': 2, 'utc': '2026-09-09T17:43:24Z'},
        {'mode': 1, 'utc': ''},
    ]}
    times, values = mode_timeline(meta, '2026-09-09T17:43:22Z')
    assert times == [2.0] and values == [2]


# ---- timeline -------------------------------------------------------------

class FakeFrame:
    def __init__(self, index, timestamp):
        self.index = index
        self.timestamp = timestamp
        self.values = {}


def timeline(n=10, dt=0.1):
    return Timeline(name='ep', task='pick',
                    frames=[FakeFrame(i, i * dt) for i in range(n)],
                    mode_times=[], mode_values=[])


def test_duration_is_the_last_frame():
    assert timeline(n=10, dt=0.1).duration == pytest.approx(0.9)


@pytest.mark.parametrize('given,expected', [(-5, 0), (0, 0), (4, 4), (99, 9)])
def test_seeking_clamps_instead_of_raising(given, expected):
    # A slider can and will ask for an out-of-range frame during a resize.
    assert timeline().clamp(given) == expected


def test_at_returns_the_clamped_frame():
    assert timeline().at(99).index == 9


def test_empty_timeline_is_safe_to_query():
    empty = Timeline(name='ep', task='', frames=[], mode_times=[],
                     mode_values=[])
    assert empty.count == 0
    assert empty.duration == 0.0
    assert empty.clamp(3) == 0
    assert empty.at(0) is None


def test_mode_teleop_matches_the_message_definition():
    # library.py hardcodes this so it needs no message dependency; if
    # TeleopStatus ever renumbers, this fails rather than mislabelling the
    # engaged portion of every episode.
    from moveit_teleop_msgs.msg import TeleopStatus

    from moveit_teleop_replay.library import MODE_TELEOP
    assert MODE_TELEOP == TeleopStatus.TELEOP


# ---- the command surface --------------------------------------------------

def test_every_command_the_player_dispatches_on_exists():
    """Guards a crash rather than an error.

    The player builds its handler map at the top of the service callback,
    OUTSIDE its try/except. A constant referenced from the wrong class is
    therefore not a failed command -- it takes the node down on the first
    button press, and the panel just goes dead.
    """
    import inspect

    from moveit_teleop_msgs.srv import ReplayCommand

    from moveit_teleop_replay import episode_player

    source = inspect.getsource(episode_player.EpisodePlayer._on_command)
    referenced = set(re.findall(r'ReplayCommand\.Request\.([A-Z_]+)', source))
    assert referenced, 'no command constants found; did the dispatch change?'
    for name in referenced:
        assert hasattr(ReplayCommand.Request, name), (
            f'{name} is dispatched on but not defined in ReplayCommand.srv')


def test_constants_are_not_on_the_service_type():
    # Pins the mistake itself: they are on Request, and reading them from the
    # service type raises AttributeError at import-of-map time.
    from moveit_teleop_msgs.srv import ReplayCommand
    assert not hasattr(ReplayCommand, 'PLAY')
    assert hasattr(ReplayCommand.Request, 'PLAY')


# ---- launch-argument isolation --------------------------------------------
# IncludeLaunchDescription SHARES the parent's launch-configuration scope, so a
# same-named argument crosses over silently. Both directions caused real bugs:
#
#   * the replay's rate_hz (0.0 = "use the episode's own rate") reached the
#     fake device, whose timer is 1.0 / rate_hz -> ZeroDivisionError at start;
#   * the teleop launch's joint_states_topic (/control/joint_states) reached
#     the player, aiming recorded playback at the LIVE robot's state topic.
#
# Neither produced a warning. These tests are the guard.

REPLAY_NODES = (
    Path(__file__).resolve().parents[1] / 'launch' / 'replay_nodes.launch.py')
TELEOP_LAUNCH = (
    Path(__file__).resolve().parents[2] / 'moveit_teleop_bringup' / 'launch'
    / 'teleop.launch.py')


def declared_args(path):
    source = path.read_text()
    return set(re.findall(r"DeclareLaunchArgument\(\s*\n?\s*'([a-z_0-9]+)'",
                          source))


def test_replay_args_cannot_collide_with_a_hosts_own():
    """Every argument the include declares is namespaced, except the ones it
    deliberately shares with its host (which robot to draw)."""
    shared = {'arm_type', 'effector_type', 'revo2_type', 'tcp_offset'}
    for name in declared_args(REPLAY_NODES) - shared:
        assert name.startswith('replay_'), (
            f'{name!r} is not namespaced; if a host launch declares the same '
            'name it will silently override this one')


def test_teleop_passes_every_replay_argument_explicitly():
    if not TELEOP_LAUNCH.exists():          # replay can be used standalone
        pytest.skip('moveit_teleop_bringup not present')
    source = TELEOP_LAUNCH.read_text()
    passed = set(re.findall(r"'(replay_[a-z_0-9]+)':", source))
    for name in declared_args(REPLAY_NODES):
        if name.startswith('replay_'):
            assert name in passed, (
                f'{name} is not passed explicitly; it would inherit whatever '
                'the teleop launch happens to have in scope')


def test_teleop_scopes_the_include():
    if not TELEOP_LAUNCH.exists():
        pytest.skip('moveit_teleop_bringup not present')
    source = TELEOP_LAUNCH.read_text()
    include = source.index('replay_nodes.launch.py')
    # A GroupAction just before it is what stops the include's own arguments
    # leaking back out into the teleop launch.
    assert 'GroupAction([' in source[:include], (
        'the replay include is not wrapped in a GroupAction, so its arguments '
        'leak into the teleop launch scope')


def test_player_refuses_live_joint_state_topics():
    from moveit_teleop_replay.episode_player import LIVE_TOPICS
    # The exact topic the collision aimed it at.
    assert '/control/joint_states' in LIVE_TOPICS
    assert '/joint_states' in LIVE_TOPICS
    assert '/replay/joint_states' not in LIVE_TOPICS


# ---- robot mismatch -------------------------------------------------------
# Nothing in a bag records which arm produced it. Replaying a Piper episode on
# a Franka preview renders NOTHING: the DisplayRobotState carries joint1..6,
# the FR3 model has fr3_joint1..7, and is_diff means the display keeps its
# default pose. The panel says an episode is loaded and the viewport is empty.

class _Frame:
    def __init__(self, names):
        self.timestamp = 0.0
        self.index = 0
        self.values = {'joint_states': type('JS', (), {'name': names})()}


def test_episode_joints_come_from_the_first_frame_that_has_them():
    from moveit_teleop_replay.episode_player import EpisodePlayer
    tl = type('T', (), {'frames': [
        type('F', (), {'values': {}})(),
        _Frame(['joint1', 'joint2']),
    ]})()
    assert EpisodePlayer._episode_joints(tl) == {'joint1', 'joint2'}


def test_an_episode_with_no_joint_states_reports_no_joints():
    from moveit_teleop_replay.episode_player import EpisodePlayer
    tl = type('T', (), {'frames': [type('F', (), {'values': {}})()]})()
    assert EpisodePlayer._episode_joints(tl) == set()


def test_piper_and_franka_joint_sets_do_not_intersect():
    # The condition the player refuses on. Stated as a test so that a future
    # naming scheme that DOES overlap is noticed here rather than in a viewport.
    piper = {'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper'}
    franka = {f'fr3_joint{i}' for i in range(1, 8)} | {'fr3_finger_joint1'}
    assert not (piper & franka)
