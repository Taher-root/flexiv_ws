"""Behavioural tests for clutched relative-pose retargeting.

The scenario these exist for is the one that ruins a teleop session: the
operator clutches out, moves their hand across the desk, clutches back in, and
the arm leaps. Every test here is a variation on proving that cannot happen.
"""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from moveit_teleop_core.retargeting.base import RetargetInput
from moveit_teleop_core.retargeting.relative_pose import RelativePoseRetargeter
from moveit_teleop_core.scene_config import Plane
from moveit_teleop_core.transforms import CONVENTIONS

EE_POS = np.array([0.3, 0.0, 0.4])
EE_QUAT = np.array([0.0, 0.0, 0.0, 1.0])


def sample(device_pos, clutch=True, device_quat=EE_QUAT, valid=True,
           ee_pos=EE_POS, ee_quat=EE_QUAT, gripper=0.0):
    return RetargetInput(
        device_position=np.asarray(device_pos, float),
        device_orientation=np.asarray(device_quat, float),
        clutch=clutch, pose_valid=valid, gripper=gripper,
        ee_position=np.asarray(ee_pos, float),
        ee_orientation=np.asarray(ee_quat, float))


def make(**cfg):
    """Build a retargeter for a test.

    The leash and jump guards default WIDE OPEN here so that tests of the
    mapping measure the mapping. Tests of the guards themselves set their own
    limits explicitly -- otherwise every mapping test doubles as an accidental
    and confusing test of the clamps.
    """
    cfg.setdefault('translation_scale', 1.0)
    cfg.setdefault('rotation_scale', 1.0)
    cfg.setdefault('max_translation_step', 100.0)
    cfg.setdefault('max_target_lead', 100.0)
    cfg.setdefault('max_target_lead_angle', 100.0)
    cfg.setdefault('external_motion_threshold', 1000.0)
    return RelativePoseRetargeter(cfg)


def test_declutched_is_inactive():
    r = make()
    out = r.update(sample([0, 0, 0], clutch=False))
    assert not out.active and out.reason == 'declutched'


def test_first_engaged_sample_anchors_at_current_ee():
    """Engaging must never move the arm, however far away the device is."""
    r = make()
    out = r.update(sample([5.0, -3.0, 2.0]))
    assert out.active and out.reason == 'anchored'
    assert np.allclose(out.position, EE_POS), 'engaging moved the arm'


def test_motion_since_anchor_is_applied():
    # max_translation_step is raised because this exercises the mapping, not
    # the jump guard; 0.1 m in one update is 10 m/s and is correctly rejected
    # by the default limit (see test_large_jump_is_rejected).
    r = make(translation_scale=1.0, max_translation_step=1.0)
    r.update(sample([0, 0, 0]))
    out = r.update(sample([0.1, 0.0, 0.0]))
    assert np.allclose(out.position, EE_POS + [0.1, 0, 0])


def test_translation_scale_is_applied():
    r = make(translation_scale=0.5, max_translation_step=1.0)
    r.update(sample([0, 0, 0]))
    out = r.update(sample([0.1, 0.0, 0.0]))
    assert np.allclose(out.position, EE_POS + [0.05, 0, 0])


def test_reengage_after_hand_moved_does_not_jump():
    """THE regression this whole design exists to prevent."""
    r = make(translation_scale=1.0, max_translation_step=1.0)
    r.update(sample([0, 0, 0]))
    moved = r.update(sample([0.05, 0, 0]))
    reached = moved.position                      # pretend the arm got there

    r.update(sample([0.05, 0, 0], clutch=False))  # clutch out
    r.update(sample([0.9, -0.7, 0.4], clutch=False))   # hand flies across desk

    out = r.update(sample([0.9, -0.7, 0.4], ee_pos=reached))
    assert out.active and out.reason == 'anchored'
    assert np.allclose(out.position, reached), \
        f'jumped {np.linalg.norm(out.position - reached):.3f} m on re-engage'


def test_reanchors_to_actual_ee_not_last_target():
    """If servo under-tracked, the next engage must start from reality."""
    r = make(translation_scale=1.0, max_translation_step=1.0)
    r.update(sample([0, 0, 0]))
    commanded = r.update(sample([0.2, 0, 0])).position
    actual = EE_POS + [0.05, 0, 0]                # servo only got 1/4 of the way
    assert not np.allclose(commanded, actual)

    r.update(sample([0.2, 0, 0], clutch=False))
    out = r.update(sample([0.2, 0, 0], ee_pos=actual))
    assert np.allclose(out.position, actual), 'anchored to the target, not reality'


def test_tracking_loss_drops_the_anchor():
    """Holding the anchor through a gap makes the arm jump when tracking returns."""
    r = make(translation_scale=1.0, max_translation_step=1.0)
    r.update(sample([0, 0, 0]))
    out = r.update(sample([0, 0, 0], valid=False))
    assert not out.active and out.reason == 'tracking lost'

    out = r.update(sample([0.6, 0.6, 0.6]))       # hand moved during the gap
    assert out.reason == 'anchored'
    assert np.allclose(out.position, EE_POS)


def test_large_jump_is_rejected():
    r = make(translation_scale=1.0, max_translation_step=0.05,
             max_target_lead=100.0)
    r.update(sample([0, 0, 0]))
    out = r.update(sample([10.0, 0, 0]))
    assert not out.active and 'jump' in out.reason


def test_clutch_storm_never_produces_a_discontinuity():
    """Toggle the clutch rapidly while moving, as the fake device does."""
    r = make(translation_scale=0.5, max_translation_step=0.05,
             max_target_lead=0.5)
    ee = EE_POS.copy()
    last = ee.copy()
    for i in range(400):
        clutch = (i // 7) % 2 == 0
        device = np.array([0.1 * np.cos(i * 0.05), 0.1 * np.sin(i * 0.05), 0.0])
        out = r.update(sample(device, clutch=clutch, ee_pos=ee))
        if out.active:
            step = np.linalg.norm(out.position - last)
            assert step < 0.05, f'discontinuity of {step:.4f} m at i={i}'
            last = out.position.copy()
            ee = out.position.copy()      # perfect tracking


def test_rotation_ref_tool_vs_base_differ():
    """A non-identity EE orientation must distinguish the two conventions."""
    ee_quat = Rotation.from_euler('z', np.pi / 2).as_quat()
    dev = Rotation.from_euler('x', 0.4).as_quat()

    outs = {}
    for ref in ('tool', 'base'):
        r = make(rotation_ref=ref, translation_scale=1.0)
        r.update(sample([0, 0, 0], ee_quat=ee_quat))
        outs[ref] = r.update(sample([0, 0, 0], device_quat=dev, ee_quat=ee_quat)).orientation

    assert not np.allclose(outs['tool'], outs['base']), \
        'tool and base rotation references produced identical results'


def test_translation_ref_tool_rotates_with_the_tool():
    ee_quat = Rotation.from_euler('z', np.pi / 2).as_quat()
    r = make(translation_ref='tool', translation_scale=1.0,
             max_translation_step=1.0)
    r.update(sample([0, 0, 0], ee_quat=ee_quat))
    out = r.update(sample([0.1, 0, 0], ee_quat=ee_quat))
    # +x in the tool frame is +y in base after a 90 deg z rotation.
    assert np.allclose(out.position, EE_POS + [0.0, 0.1, 0.0], atol=1e-9)


def test_reset_clears_everything():
    r = make(translation_scale=1.0)
    r.update(sample([0, 0, 0]))
    r.update(sample([0.1, 0, 0]))
    r.reset()
    out = r.update(sample([0.9, 0, 0]))
    assert out.reason == 'anchored' and np.allclose(out.position, EE_POS)


def test_gripper_passes_through():
    r = make()
    assert r.update(sample([0, 0, 0], gripper=0.75)).gripper == pytest.approx(0.75)


def test_rejects_bad_frame_refs():
    with pytest.raises(ValueError, match='translation_ref'):
        make(translation_ref='elbow')


def test_external_motion_forces_reanchor():
    """Something else moved the robot: re-anchor instead of commanding a lurch.

    This is the startup race that broke closed-loop bring-up: the retargeter
    anchored while the arm was still at its home pose, the arm then moved to
    ready_pose, and the retargeter kept commanding a path around where the arm
    used to be -- 12 cm away, which servo answered with an emergency stop.

    It is the same mechanism that covers the planned-motion handoff, a manual
    jog, and anything else that moves the arm behind our back.
    """
    r = make(translation_scale=1.0, max_translation_step=1.0,
             max_target_lead=0.02, external_motion_threshold=0.05)
    r.update(sample([0, 0, 0]))
    r.update(sample([0.02, 0, 0]))

    # The arm is teleported somewhere we never commanded.
    moved_ee = EE_POS + [0.3, 0.0, 0.0]
    out = r.update(sample([0.02, 0, 0], ee_pos=moved_ee))

    assert out.reason == 'anchored', f'did not re-anchor, said {out.reason!r}'
    assert np.allclose(out.position, moved_ee), 'anchored somewhere other than reality'


def test_small_tracking_error_does_not_trigger_reanchor():
    """Normal servo lag must not be mistaken for external motion.

    If it were, the retargeter would re-anchor constantly during fast motion
    and the arm would never actually follow the operator.
    """
    r = make(translation_scale=1.0, max_translation_step=1.0,
             max_target_lead=0.04, external_motion_threshold=0.05)
    r.update(sample([0, 0, 0]))
    out = r.update(sample([0.03, 0, 0]))
    assert out.reason == 'tracking'

    lagging_ee = out.position - [0.01, 0, 0]      # 1 cm behind, well within
    out = r.update(sample([0.04, 0, 0], ee_pos=lagging_ee))
    assert out.reason == 'tracking', f're-anchored on normal lag ({out.reason})'


# ---- world vs body frame: the "doesn't feel right" bugs --------------------

def test_translation_is_world_referenced_regardless_of_wrist_rotation():
    """THE feel bug: pushing forward must move the arm forward, always.

    Previously the delta was taken in the controller's BODY frame, so engaging
    the clutch with the wrist rotated re-mapped every direction -- push forward
    and the arm went sideways, differently after every re-anchor.
    """
    device_moves = np.array([0.05, 0.0, 0.0])          # push along device +x

    results = []
    for wrist in (0.0, np.pi / 2, np.pi):              # engage at three rolls
        q = Rotation.from_euler('z', wrist).as_quat()
        r = make(translation_scale=1.0, max_translation_step=1.0)
        r.update(sample([0, 0, 0], device_quat=q))
        out = r.update(sample(device_moves, device_quat=q))
        results.append(out.position - EE_POS)

    for got in results[1:]:
        assert np.allclose(got, results[0], atol=1e-9), (
            f'wrist orientation at engage changed the translation mapping: '
            f'{results}')
    assert np.allclose(results[0], [0.05, 0, 0], atol=1e-9)


def test_base_rotation_matches_the_hand_turn_in_the_room():
    """Turn the hand 30 deg about world z -> the tool turns 30 deg about world z."""
    r = make(rotation_ref='base', rotation_scale=1.0, max_translation_step=1.0)
    ee_quat = Rotation.from_euler('y', 0.7).as_quat()      # arbitrary tool pose
    r.update(sample([0, 0, 0], ee_quat=ee_quat))

    turn = Rotation.from_euler('z', 0.5)
    out = r.update(sample([0, 0, 0], device_quat=turn.as_quat(), ee_quat=ee_quat))

    expected = (turn * Rotation.from_quat(ee_quat)).as_matrix()
    assert np.allclose(Rotation.from_quat(out.orientation).as_matrix(),
                       expected, atol=1e-9)


def test_base_rotation_is_independent_of_tool_orientation():
    """A world-referenced turn must not depend on how the tool happens to sit."""
    turn = Rotation.from_euler('x', 0.4)
    deltas = []
    for tool in (0.0, 1.0, -2.0):
        ee_quat = Rotation.from_euler('z', tool).as_quat()
        r = make(rotation_ref='base', max_translation_step=1.0)
        r.update(sample([0, 0, 0], ee_quat=ee_quat))
        out = r.update(sample([0, 0, 0], device_quat=turn.as_quat(), ee_quat=ee_quat))
        # Relative to its own anchor, each must have turned by the same amount.
        rel = Rotation.from_quat(out.orientation) * Rotation.from_quat(ee_quat).inv()
        deltas.append(rel.as_rotvec())
    for d in deltas[1:]:
        assert np.allclose(d, deltas[0], atol=1e-9), deltas
    assert np.allclose(deltas[0], turn.as_rotvec(), atol=1e-9)


def test_rotation_scale_halves_the_angle():
    r = make(rotation_ref='base', rotation_scale=0.5, max_translation_step=1.0)
    r.update(sample([0, 0, 0]))
    turn = Rotation.from_euler('z', 1.0)
    out = r.update(sample([0, 0, 0], device_quat=turn.as_quat()))
    got = Rotation.from_quat(out.orientation).as_rotvec()
    assert np.allclose(got, [0, 0, 0.5], atol=1e-9)


def test_webxr_convention_maps_hand_turn_into_ros_axes():
    """A turn about WebXR 'up' must become a turn about ROS +z."""
    r = make(rotation_ref='base', device_convention='webxr',
             max_translation_step=1.0)
    r.update(sample([0, 0, 0]))
    turn = Rotation.from_rotvec([0, 0.6, 0])      # about WebXR up (+y)
    out = r.update(sample([0, 0, 0], device_quat=turn.as_quat()))
    assert np.allclose(Rotation.from_quat(out.orientation).as_rotvec(),
                       [0, 0, 0.6], atol=1e-9)


def test_webxr_convention_maps_hand_push_into_ros_axes():
    """Pushing along WebXR -z (forward) must move the tool along ROS +x."""
    r = make(translation_scale=1.0, device_convention='webxr',
             max_translation_step=1.0)
    r.update(sample([0, 0, 0]))
    out = r.update(sample([0.0, 0.0, -0.04]))     # forward in WebXR
    assert np.allclose(out.position - EE_POS, [0.04, 0, 0], atol=1e-9)


# ---- the leash: what stops the target running away from a blocked arm ------

def test_target_never_leads_the_arm_by_more_than_the_leash():
    """Simulate an arm that is completely stuck (joint limit).

    The operator keeps moving; the target must stop advancing rather than
    winding off into space.
    """
    r = make(translation_scale=1.0, max_target_lead=0.05,
             external_motion_threshold=0.15)
    stuck_ee = EE_POS.copy()
    r.update(sample([0, 0, 0], ee_pos=stuck_ee))

    for i in range(1, 60):                       # hand travels 30 cm
        out = r.update(sample([0.005 * i, 0, 0], ee_pos=stuck_ee))
        if out.active:
            lead = np.linalg.norm(out.position - stuck_ee)
            assert lead <= 0.05 + 1e-9, f'target ran {lead:.3f} m ahead at i={i}'


def test_blocked_arm_does_not_cause_reanchor_flapping():
    """The regression from the live session: anchored/tracking/anchored/...

    A stuck arm diverges from the target; with the leash bounding that
    divergence below the external-motion threshold, the retargeter must keep
    tracking instead of re-anchoring every few frames.
    """
    r = make(translation_scale=1.0, max_target_lead=0.05,
             external_motion_threshold=0.15)
    stuck_ee = EE_POS.copy()
    r.update(sample([0, 0, 0], ee_pos=stuck_ee))

    reasons = []
    for i in range(1, 80):
        out = r.update(sample([0.004 * i, 0, 0], ee_pos=stuck_ee))
        reasons.append(out.reason)
    assert 'anchored' not in reasons, (
        f're-anchored {reasons.count("anchored")} times while merely blocked')


def test_target_catches_up_once_the_arm_frees(): 
    """The leash clamps the OUTPUT only; the anchor is untouched, so the full
    commanded pose returns as soon as the arm moves again."""
    r = make(translation_scale=1.0, max_target_lead=0.05,
             external_motion_threshold=0.15)
    r.update(sample([0, 0, 0]))
    blocked = r.update(sample([0.20, 0, 0], ee_pos=EE_POS)).position
    assert np.linalg.norm(blocked - EE_POS) == pytest.approx(0.05, abs=1e-9)

    # Arm has now travelled most of the way; the same hand pose must command
    # the full offset rather than staying clamped.
    freed_ee = EE_POS + [0.18, 0, 0]
    freed = r.update(sample([0.20, 0, 0], ee_pos=freed_ee)).position
    assert np.allclose(freed, EE_POS + [0.20, 0, 0], atol=1e-9)


def test_rotation_is_leashed_too():
    r = make(rotation_ref='base', rotation_scale=1.0,
             max_target_lead=1.0, max_target_lead_angle=0.5,
             external_motion_threshold=1.5)
    r.update(sample([0, 0, 0]))
    turn = Rotation.from_euler('z', 2.5)          # far beyond the leash
    out = r.update(sample([0, 0, 0], device_quat=turn.as_quat()))
    angle = Rotation.from_quat(out.orientation).magnitude()
    assert angle == pytest.approx(0.5, abs=1e-9), f'rotation lead was {angle}'


def test_leash_must_be_smaller_than_the_external_motion_threshold():
    """Otherwise leashed lag is misread as a teleport -- the flapping bug."""
    with pytest.raises(ValueError, match='must exceed'):
        make(max_target_lead=0.2, external_motion_threshold=0.1)


# ---- operator-referenced translation --------------------------------------
# Reported from the headset: "if I turn my whole body around including the
# controller and push forward, the robot moves backward." That is correct
# behaviour for a delta measured in the ROOM, and wrong for what a hand means
# by "forward" -- the room does not turn when the operator does.

WEBXR = CONVENTIONS['webxr']


def _facing(heading_deg):
    """Device orientation, as a quaternion in WebXR axes, for a ROS heading."""
    R_ros = Rotation.from_euler('z', heading_deg, degrees=True).as_matrix()
    return Rotation.from_matrix(WEBXR.T @ R_ros @ WEBXR).as_quat()


def _push(retargeter, heading_deg, operator_frame_push):
    """Face `heading_deg`, engage, then push along the operator's own axes."""
    quat = _facing(heading_deg)
    R_ros = Rotation.from_euler('z', heading_deg, degrees=True).as_matrix()
    # The same physical push is a different vector in the room for each heading.
    dp_room_ros = R_ros @ np.asarray(operator_frame_push, float)
    dp_device = WEBXR.T @ dp_room_ros

    retargeter.update(sample([0, 0, 0], device_quat=quat))
    out = retargeter.update(sample(dp_device, device_quat=quat))
    return np.asarray(out.position) - EE_POS


def _operator_retargeter(ref):
    return make(translation_ref=ref, device_convention='webxr')


def test_base_reference_reverses_when_you_turn_around():
    """The reported bug, pinned so a regression is unambiguous."""
    ahead = _push(_operator_retargeter('base'), 0, [0.1, 0, 0])
    behind = _push(_operator_retargeter('base'), 180, [0.1, 0, 0])
    assert ahead[0] == pytest.approx(0.1, abs=1e-6)
    assert behind[0] == pytest.approx(-0.1, abs=1e-6)   # wrong way


@pytest.mark.parametrize('heading', [0, 45, 90, 180, -120])
def test_device_reference_is_unchanged_by_which_way_you_face(heading):
    moved = _push(_operator_retargeter('device'), heading, [0.1, 0, 0])
    assert moved[0] == pytest.approx(0.1, abs=1e-6)
    assert abs(moved[1]) < 1e-6 and abs(moved[2]) < 1e-6


@pytest.mark.parametrize('heading', [0, 90, 180, -60])
def test_device_reference_keeps_left_and_up_true(heading):
    left = _push(_operator_retargeter('device'), heading, [0.0, 0.1, 0.0])
    assert left[1] == pytest.approx(0.1, abs=1e-6)
    up = _push(_operator_retargeter('device'), heading, [0.0, 0.0, 0.1])
    assert up[2] == pytest.approx(0.1, abs=1e-6)


def test_heading_ignores_how_the_controller_is_tilted():
    """Holding a controller angled steeply down is completely ordinary.

    An Euler-yaw heading is ill-conditioned there; the twist decomposition is
    not. Pitch must not leak into the horizontal mapping.
    """
    R_ros = (Rotation.from_euler('z', 90, degrees=True)
             * Rotation.from_euler('y', 85, degrees=True)).as_matrix()
    quat = Rotation.from_matrix(WEBXR.T @ R_ros @ WEBXR).as_quat()
    r = _operator_retargeter('device')
    r.update(sample([0, 0, 0], device_quat=quat))
    # Push north in the room. Facing east, that is the operator's right.
    out = r.update(sample(WEBXR.T @ np.array([0.1, 0.0, 0.0]), device_quat=quat))
    moved = np.asarray(out.position) - EE_POS
    assert moved[1] == pytest.approx(-0.1, abs=1e-6)
    assert abs(moved[2]) < 1e-6, 'pitch leaked into the horizontal mapping'


def test_heading_is_frozen_for_the_duration_of_the_clutch():
    """Recomputing per sample would rotate the mapping mid-motion, so a
    straight push would curve away under the operator's hand."""
    r = _operator_retargeter('device')
    r.update(sample([0, 0, 0], device_quat=_facing(0)))
    out = r.update(sample(WEBXR.T @ np.array([0.1, 0.0, 0.0]),
                          device_quat=_facing(90)))   # turned mid-push
    moved = np.asarray(out.position) - EE_POS
    assert moved[0] == pytest.approx(0.1, abs=1e-6)


def test_re_engaging_picks_up_the_new_heading():
    r = _operator_retargeter('device')
    r.update(sample([0, 0, 0], device_quat=_facing(0)))
    r.update(sample([0, 0, 0], device_quat=_facing(0), clutch=False))
    moved = _push(r, 180, [0.1, 0, 0])
    assert moved[0] == pytest.approx(0.1, abs=1e-6)


def test_unknown_reference_frame_is_rejected():
    with pytest.raises(ValueError, match='base'):
        make(translation_ref='world')


# ---- virtual walls --------------------------------------------------------
# Reported from the headset: with the target under the table the arm would not
# slide, and bringing the hand back out somewhere else made the arm jump.
#
# Both are the LEASH. It clamps the target onto a SPHERE around the end
# effector, so a target half a metre below the table becomes a point 0.3 m away
# in a mostly-downward direction: sliding the hand sideways barely rotates that
# vector, and the accumulated distance gets covered at full rate the moment the
# target is reachable again. Projecting onto the wall BEFORE leashing puts the
# target on the table under the hand instead.

# Walls are scene_config.Plane objects, which carry a footprint. Unbounded
# here so these tests measure the projection, not the footprint; bounded
# behaviour is covered in test_scene_config.py.
TABLE_WALL = [Plane(name='table:+z', frame='world',
                    normal=np.array([0.0, 0.0, 1.0]), offset=-0.005)]


def walled(**cfg):
    r = make(**cfg)
    r.walls = list(TABLE_WALL)
    return r


def test_a_target_above_the_wall_is_untouched():
    r = walled()
    r.update(sample([0, 0, 0]))
    out = r.update(sample([0.05, 0.0, 0.05]))
    assert out.position[2] == pytest.approx(EE_POS[2] + 0.05)
    assert r.contacts == []


def test_a_target_below_the_wall_is_projected_onto_it():
    r = walled(max_target_lead=100.0)
    r.update(sample([0, 0, 0]))
    out = r.update(sample([0.0, 0.0, -1.0]))
    assert out.position[2] == pytest.approx(-0.005)
    assert r.contacts == ['table:+z']


def test_projection_keeps_the_lateral_component():
    """The property that makes the tool slide."""
    r = walled(max_target_lead=100.0)
    r.update(sample([0, 0, 0]))
    a = r.update(sample([0.10, 0.05, -1.0])).position
    b = r.update(sample([0.30, 0.05, -1.0])).position
    assert b[0] - a[0] == pytest.approx(0.20)
    assert a[1] == pytest.approx(b[1])
    assert a[2] == b[2] == pytest.approx(-0.005)


def test_contact_depth_is_the_distance_past_the_surface():
    r = walled(max_target_lead=100.0)
    r.update(sample([0, 0, 0]))
    # EE starts at z=0.4, so a device delta of -0.5 puts the target at -0.1,
    # which is 0.095 below the surface at -0.005.
    r.update(sample([0.0, 0.0, -0.5]))
    assert r.contact_depth == pytest.approx(0.095, abs=1e-6)


def test_contact_clears_when_the_target_comes_back_up():
    r = walled(max_target_lead=100.0)
    r.update(sample([0, 0, 0]))
    r.update(sample([0.0, 0.0, -1.0]))
    assert r.contacts
    r.update(sample([0.0, 0.0, 0.1]))
    assert r.contacts == []
    assert r.contact_depth == 0.0


def test_no_walls_means_no_projection_and_no_contact():
    r = make(max_target_lead=100.0)
    r.update(sample([0, 0, 0]))
    out = r.update(sample([0.0, 0.0, -1.0]))
    assert out.position[2] == pytest.approx(EE_POS[2] - 1.0)
    assert r.contacts == []


def test_the_tool_slides_along_the_wall_in_closed_loop():
    """The end-to-end behaviour, with the arm actually following.

    Pinning the end effector would hide the fix: the leash legitimately holds
    the target back until the arm descends, so only a loop where the arm
    catches up shows where it settles.
    """
    r = walled(max_target_lead=0.30)
    ee = np.array(EE_POS, float)

    def step(hand):
        nonlocal ee
        for _ in range(60):
            out = r.update(sample(hand, ee_pos=ee))
            ee = ee + 0.25 * (np.asarray(out.position) - ee)
        return ee.copy()

    r.update(sample([0, 0, 0]))
    step([0.0, 0.0, -0.6])
    positions = [step([x, 0.0, -0.6]) for x in (0.10, 0.20, 0.30)]

    for p in positions:
        assert p[2] == pytest.approx(-0.005, abs=1e-3), 'tool left the surface'
    # 10 cm of hand motion is 10 cm of tool motion, along the surface.
    assert positions[1][0] - positions[0][0] == pytest.approx(0.10, abs=1e-3)
    assert positions[2][0] - positions[1][0] == pytest.approx(0.10, abs=1e-3)


def test_without_walls_the_tool_goes_straight_through():
    """The behaviour being replaced, pinned so the difference is explicit."""
    r = make(max_target_lead=0.30)
    ee = np.array(EE_POS, float)
    r.update(sample([0, 0, 0]))
    for _ in range(60):
        out = r.update(sample([0.0, 0.0, -0.6], ee_pos=ee))
        ee = ee + 0.25 * (np.asarray(out.position) - ee)
    assert ee[2] < -0.1, 'expected the target to sink below the table'


# ---- wall discontinuities -------------------------------------------------
# Projection fixed the sliding but left a jump at the moment a wall STOPS
# applying: reach past the edge of a table and the command snaps from the table
# surface down to wherever the hand actually is. Nothing moved but the wall.
#
# The answer is to re-anchor, not to reject. Rejecting keeps the stale anchor,
# so the divergence survives and the arm lurches as soon as the target is
# commandable again -- the original bug wearing a different hat.

BOUNDED_TABLE = [Plane(name='table:+z', frame='world',
                       normal=np.array([0.0, 0.0, 1.0]), offset=0.025,
                       extent=np.array([-0.4, 0.4, -0.4, 0.4]))]


def guarded(step=0.02, walls=True, **cfg):
    cfg.setdefault('max_translation_step', 0.05)
    cfg.setdefault('max_target_lead', 100.0)
    r = make(max_command_step=step, **cfg)
    if walls:
        r.walls = list(BOUNDED_TABLE)
    return r


def walk(retargeter, frm, to, ee_pos=None, n=60):
    """Move the hand there in realistic increments.

    Single large steps trip the DEVICE guard instead, which is a different
    mechanism -- these tests are about what a wall does under a hand that is
    moving normally.
    """
    frm, to = np.asarray(frm, float), np.asarray(to, float)
    out = None
    for k in range(1, n + 1):
        p = frm + (to - frm) * k / n
        kw = {} if ee_pos is None else {'ee_pos': ee_pos}
        out = retargeter.update(sample(p, **kw))
    return out


def reasons(retargeter, frm, to, n=60):
    frm, to = np.asarray(frm, float), np.asarray(to, float)
    seen = []
    for k in range(1, n + 1):
        seen.append(retargeter.update(
            sample(frm + (to - frm) * k / n)).reason)
    return seen


def test_leaving_the_footprint_reanchors_instead_of_jumping():
    r = guarded()
    r.update(sample([0, 0, 0]))
    walk(r, [0, 0, 0], [0.0, 0.0, -0.40])          # under the table
    assert r.update(sample([0.0, 0.0, -0.40])).position[2] == pytest.approx(0.025)
    # Walk sideways until the target passes the footprint edge.
    seen = reasons(r, [0.0, 0.0, -0.40], [0.30, 0.0, -0.40])
    assert any('re-anchored' in x for x in seen), 'no re-anchor at the edge'


def test_the_arm_does_not_move_on_the_re_anchoring_update():
    r = guarded()
    r.update(sample([0, 0, 0]))
    walk(r, [0, 0, 0], [0.0, 0.0, -0.40])
    frm, to = np.array([0.0, 0.0, -0.40]), np.array([0.30, 0.0, -0.40])
    for k in range(1, 61):
        out = r.update(sample(frm + (to - frm) * k / 60))
        if 're-anchored' in out.reason:
            assert np.allclose(out.position, EE_POS)
            return
    pytest.fail('never re-anchored')


def test_reanchoring_rezeroes_the_mapping():
    """After a re-anchor the operator drives on from where the arm IS."""
    r = guarded()
    r.update(sample([0, 0, 0]))
    walk(r, [0, 0, 0], [0.0, 0.0, -0.40])
    frm, to = np.array([0.0, 0.0, -0.40]), np.array([0.30, 0.0, -0.40])
    for k in range(1, 61):
        here = frm + (to - frm) * k / 60
        if 're-anchored' in r.update(sample(here)).reason:
            out = r.update(sample(here + np.array([0.01, 0.0, 0.0])))
            assert out.reason == 'tracking'
            assert out.position[0] == pytest.approx(EE_POS[0] + 0.01, abs=1e-6)
            return
    pytest.fail('never re-anchored')


def test_the_command_never_jumps_once_the_guard_is_on():
    """The largest SINGLE-UPDATE command step while TRACKING.

    Re-anchor updates are excluded on purpose, and not to flatter the result: a
    re-anchor commands the arm's own current pose, which cannot move it however
    far that is from the previous command. Counting it would measure the arm's
    tracking lag, not a jump.
    """
    def worst(step):
        r = guarded(step=step)
        r.update(sample([0, 0, 0]))
        prev, biggest = None, 0.0
        # Deep enough under the wall that losing it is a real jump: the
        # anchor is at z=0.4 and the wall at 0.025, so a hand 0.75 m down puts
        # the target 0.375 m below the surface.
        for frm, to in (([0, 0, 0], [0.0, 0.0, -0.75]),
                        ([0.0, 0.0, -0.75], [0.60, 0.0, -0.75]),
                        ([0.60, 0.0, -0.75], [0.60, 0.0, 0.10])):
            frm, to = np.asarray(frm, float), np.asarray(to, float)
            for k in range(1, 81):
                out = r.update(sample(frm + (to - frm) * k / 80))
                cmd = np.asarray(out.position)
                if prev is not None and 're-anchored' not in out.reason:
                    biggest = max(biggest, float(np.linalg.norm(cmd - prev)))
                prev = cmd
        return biggest

    assert worst(0.0) > 0.25, 'expected the old behaviour to jump'
    assert worst(0.02) < 0.03, 'the guard did not stop the jump'


def test_ordinary_fast_motion_never_reanchors():
    """The guard measures EXCESS over hand motion, so speed alone is fine."""
    r = guarded(walls=False)
    r.update(sample([0, 0, 0]))
    assert all(x == 'tracking'
               for x in reasons(r, [0, 0, 0], [0.9, 0.0, 0.3], n=20))


def test_the_leash_releasing_never_reanchors():
    """The arm catching up moves the command further than the hand did, and
    that is ordinary tracking -- which is why the check runs before the leash."""
    r = guarded(walls=False, max_target_lead=0.05)
    ee = np.array(EE_POS, float)
    r.update(sample([0, 0, 0], ee_pos=ee))
    for k in range(1, 300):
        hand = np.array([min(0.4, k * 0.004), 0.0, 0.0])
        out = r.update(sample(hand, ee_pos=ee))
        assert out.reason == 'tracking', f're-anchored on leash release at {k}'
        ee = ee + 0.1 * (np.asarray(out.position) - ee)


def test_sliding_along_a_wall_never_reanchors():
    r = guarded()
    r.update(sample([0, 0, 0]))
    walk(r, [0, 0, 0], [0.0, 0.0, -0.40])
    # Stays well inside the footprint throughout.
    assert all(x == 'tracking'
               for x in reasons(r, [0.0, 0.0, -0.40], [0.05, 0.0, -0.40]))


def test_the_guard_can_be_switched_off():
    r = guarded(step=0.0)
    r.update(sample([0, 0, 0]))
    walk(r, [0, 0, 0], [0.0, 0.0, -0.40])
    seen = reasons(r, [0.0, 0.0, -0.40], [0.30, 0.0, -0.40])
    assert all('re-anchored' not in x for x in seen)
