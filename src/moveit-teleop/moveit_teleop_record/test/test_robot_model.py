"""The exporter learns a robot's joint layout from its MoveIt model, not from us."""


import pytest

from moveit_teleop_record.robot_model import (
    DATASET_DEFAULTS, RobotConfigError, dataset_settings, load_robot_model,
)


# ---- the dataset section --------------------------------------------------

def test_an_absent_section_gives_the_defaults():
    assert dataset_settings(None) == DATASET_DEFAULTS


def test_a_partial_section_merges_over_the_defaults():
    s = dataset_settings({'robot_type': 'x', 'gr00t': {'action_horizon': 8}})
    assert s['robot_type'] == 'x'
    assert s['gr00t']['action_horizon'] == 8
    assert s['gr00t']['arm_action'] == 'relative'      # untouched default
    assert DATASET_DEFAULTS['gr00t']['action_horizon'] == 16   # not mutated


def test_a_misspelled_key_is_refused():
    # The teleop launch ignores this section, so nothing else would notice.
    with pytest.raises(RobotConfigError, match='does not read'):
        dataset_settings({'image_sise': '640x360'})


def test_an_invalid_choice_is_refused():
    with pytest.raises(RobotConfigError, match='arm_action'):
        dataset_settings({'gr00t': {'arm_action': 'sideways'}})


def test_a_bad_horizon_is_refused():
    with pytest.raises(RobotConfigError, match='action_horizon'):
        dataset_settings({'gr00t': {'action_horizon': 0}})


# ---- required fields ------------------------------------------------------

def _write(tmp_path, body):
    p = tmp_path / 'teleop.yaml'
    p.write_text(body)
    return p


def test_naming_no_robot_says_how():
    with pytest.raises(RobotConfigError, match='--robot-config-package'):
        load_robot_model(None, None)


def test_missing_move_group_is_named_as_required(tmp_path):
    cfg = _write(tmp_path, 'moveit_teleop:\n  ready_state: ready_pose\n')
    with pytest.raises(RobotConfigError, match='`move_group` is REQUIRED'):
        load_robot_model(path=cfg)


def test_missing_model_files_are_named_as_required(tmp_path):
    cfg = _write(tmp_path, 'moveit_teleop:\n  move_group: arm\n')
    with pytest.raises(RobotConfigError, match='files.preview_urdf` is REQUIRED'):
        load_robot_model(path=cfg)


def test_a_missing_model_file_names_the_path(tmp_path):
    cfg = _write(tmp_path, 'moveit_teleop:\n  move_group: arm\n  files:\n'
                           '    preview_urdf: nope.urdf\n    preview_srdf: nope.srdf\n')
    with pytest.raises(RobotConfigError, match='nope.urdf'):
        load_robot_model(path=cfg)


# ---- the two shipped robots, from their real configs ----------------------
# Needs the robots' MoveIt config packages and xacro, so it runs where the
# workspace is built and is skipped elsewhere.

def _have(package):
    """Is this robot's config installed here?

    Checks the config file, not just the package: a machine can legitimately
    have an older copy of an optional robot installed without it, and that is
    a missing robot, not a failing exporter.
    """
    try:
        import os
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory(package)
        return os.path.exists(os.path.join(share, 'config', 'teleop.yaml'))
    except Exception:  # noqa: BLE001
        return False


@pytest.mark.skipif(not _have('agx_arm_moveit'), reason='agx_arm_moveit not built')
def test_piper_layout_comes_from_its_srdf():
    m = load_robot_model('agx_arm_moveit')
    d = m.describe()
    assert m.name == 'agx_arm' and m.robot_type == 'agx_piper'
    assert [(g['name'], g['srdf_group'], g['start'], g['end']) for g in d['groups']] == [
        ('arm', 'arm', 0, 6), ('gripper', 'gripper', 6, 7)]
    assert m.joint_names == ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
    assert d['groups'][0]['units'] == ['rad'] * 6 and d['groups'][1]['units'] == ['m']


@pytest.mark.skipif(not _have('franka_teleop_bringup'), reason='franka_teleop_bringup not built')
def test_franka_layout_comes_from_its_srdf():
    # No joint here contains "gripper": the old name-guessing would have put the
    # finger in the arm group.
    m = load_robot_model('franka_teleop_bringup')
    d = m.describe()
    assert m.name == 'fr3' and m.robot_type == 'franka_fr3'
    assert [(g['name'], g['start'], g['end']) for g in d['groups']] == [
        ('arm', 0, 7), ('gripper', 7, 8)]
    assert d['groups'][1]['joints'] == ['fr3_finger_joint1']
