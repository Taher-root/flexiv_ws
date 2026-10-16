"""Tests for the planning-scene declaration.

The geometry here decides where the arm is allowed to be, so it is worth
testing on its own rather than only through a running stack.
"""

import numpy as np
import pytest
import yaml

from moveit_teleop_core.scene_config import (
    Plane, SceneConfig, SceneConfigError,
)

TABLE = {
    'name': 'table', 'frame': 'base_link',
    'bounds': [-0.4, 0.4, -0.4, 0.4, -0.025, -0.005],
    'wall': '+z',
}


def cfg(**over):
    raw = {'boxes': [dict(TABLE, **over)]}
    return SceneConfig.from_dict(raw)


# ---- boxes ----------------------------------------------------------------

def test_size_and_centre_come_from_the_bounds():
    box = cfg().boxes[0]
    assert np.allclose(box.size, [0.8, 0.8, 0.02])
    assert np.allclose(box.center, [0.0, 0.0, -0.015])


def test_offset_box_centres_correctly():
    box = cfg(bounds=[0.2, 0.6, -0.1, 0.1, 0.0, 0.4]).boxes[0]
    assert np.allclose(box.size, [0.4, 0.2, 0.4])
    assert np.allclose(box.center, [0.4, 0.0, 0.2])


def test_inverted_bounds_are_rejected():
    with pytest.raises(SceneConfigError, match='zmax'):
        cfg(bounds=[-0.4, 0.4, -0.4, 0.4, 0.0, -0.1])


def test_unknown_wall_face_is_rejected():
    with pytest.raises(SceneConfigError, match='wall is'):
        cfg(wall='up')


def test_duplicate_names_are_rejected():
    # A repeated id silently REPLACES the earlier object in the scene.
    with pytest.raises(SceneConfigError, match='duplicate'):
        SceneConfig.from_dict({'boxes': [TABLE, dict(TABLE)]})


# ---- the wall is derived from the box -------------------------------------
# This is the whole point of the file. Configured independently, the wall
# height and the box height drift apart and the tool halts in mid-air well
# above the table with nothing visibly wrong -- which is exactly what happened
# with the separate floor_z / tool_floor_z arguments this replaces.

def test_wall_sits_exactly_on_the_box_face():
    box = cfg().boxes[0]
    plane = box.wall_plane()
    assert np.allclose(plane.normal, [0, 0, 1])
    # zmax of the box, so the constraint z >= -0.005 IS the table top.
    assert plane.offset == pytest.approx(-0.005)


def test_wall_cannot_drift_from_the_collision_box():
    for zmax in (-0.005, 0.0, 0.25, 1.0):
        box = cfg(bounds=[-0.4, 0.4, -0.4, 0.4, zmax - 0.02, zmax]).boxes[0]
        top = box.center[2] + box.size[2] / 2
        assert box.wall_plane().offset == pytest.approx(top)


@pytest.mark.parametrize('face,normal,offset', [
    ('+z', [0, 0, 1], -0.005),
    ('-z', [0, 0, -1], 0.025),
    ('+x', [1, 0, 0], 0.4),
    ('-x', [-1, 0, 0], 0.4),
    ('+y', [0, 1, 0], 0.4),
    ('-y', [0, -1, 0], 0.4),
])
def test_every_face_yields_the_right_half_space(face, normal, offset):
    plane = cfg(wall=face).boxes[0].wall_plane()
    assert np.allclose(plane.normal, normal)
    assert plane.offset == pytest.approx(offset)


def test_a_box_without_a_wall_contributes_no_plane():
    scene = cfg(wall='')
    assert scene.all_planes() == []
    assert len(scene.collision_boxes()) == 1


def test_all_planes_includes_both_sources():
    scene = SceneConfig.from_dict({
        'boxes': [TABLE],
        'planes': [{'name': 'back', 'frame': 'base_link',
                    'normal': [1, 0, 0], 'offset': -0.25}],
    })
    assert sorted(p.name for p in scene.all_planes()) == ['back', 'table:+z']


# ---- frame changes --------------------------------------------------------

def test_plane_transform_is_identity_for_an_identity_transform():
    p = Plane('p', 'a', np.array([0.0, 0.0, 1.0]), -0.005)
    out = p.transformed(np.eye(3), [0.0, 0.0, 0.0])
    assert np.allclose(out.normal, [0, 0, 1])
    assert out.offset == pytest.approx(-0.005)


def test_plane_transform_follows_a_translation():
    # A table 5 mm below base_link, with base_link 0.1 m up in world, is a
    # table at z = 0.095 in world.
    p = Plane('p', 'base_link', np.array([0.0, 0.0, 1.0]), -0.005)
    out = p.transformed(np.eye(3), [0.0, 0.0, 0.1])
    assert out.offset == pytest.approx(0.095)


def test_plane_transform_rotates_the_normal():
    from scipy.spatial.transform import Rotation
    R = Rotation.from_euler('y', 90, degrees=True).as_matrix()
    p = Plane('p', 'a', np.array([0.0, 0.0, 1.0]), 0.0)
    out = p.transformed(R, [0.0, 0.0, 0.0])
    assert np.allclose(out.normal, [1, 0, 0], atol=1e-9)


def test_plane_transform_keeps_the_same_points_feasible():
    from scipy.spatial.transform import Rotation
    rng = np.random.default_rng(0)
    R = Rotation.random(random_state=3).as_matrix()
    t = rng.normal(size=3)
    p = Plane('p', 'a', np.array([0.2, -0.9, 0.4]), 0.3)
    q = p.transformed(R, t)
    for _ in range(200):
        local = rng.normal(size=3)
        world = R @ local + t
        assert ((p.normal @ local >= p.offset)
                == (q.normal @ world >= q.offset - 1e-12))


# ---- file loading ---------------------------------------------------------

def test_missing_root_key_is_rejected(tmp_path):
    path = tmp_path / 'scene.yaml'
    path.write_text(yaml.safe_dump({'something': {}}))
    with pytest.raises(SceneConfigError, match='planning_scene'):
        SceneConfig.from_file(path)


def test_empty_scene_is_valid(tmp_path):
    path = tmp_path / 'scene.yaml'
    path.write_text(yaml.safe_dump({'planning_scene': {}}))
    scene = SceneConfig.from_file(path)
    assert scene.boxes == [] and scene.planes == []


def test_default_frame_applies_to_entries_without_one():
    scene = SceneConfig.from_dict({
        'default_frame': 'world',
        'boxes': [{'name': 'b', 'bounds': [0, 1, 0, 1, 0, 1]}],
    })
    assert scene.boxes[0].frame == 'world'


# ---- projection -----------------------------------------------------------
# Reported from the headset: with the target under the table the arm would not
# slide, and bringing the hand back out somewhere else made the arm jump.
#
# Both came from the LEASH, which clamps the target onto a sphere around the
# end effector. A target 0.5 m below the table is clamped to 0.3 m away in a
# mostly-downward direction, so lateral hand motion barely rotates that vector
# (no slide), and the accumulated distance is covered at full rate the moment
# the target becomes reachable (the jump).
#
# Projecting onto the wall FIRST puts the target on the table under the hand:
# close to the arm, so the leash never bites, and free to move tangentially.

from moveit_teleop_core.scene_config import project, violations

TABLE_PLANE = [(np.array([0.0, 0.0, 1.0]), -0.005)]


def test_a_feasible_point_is_untouched():
    p = np.array([0.3, 0.1, 0.4])
    assert np.allclose(project(p, TABLE_PLANE), p)


def test_projection_lands_exactly_on_the_surface():
    out = project([0.3, 0.1, -0.5], TABLE_PLANE)
    assert out[2] == pytest.approx(-0.005)


def test_projection_preserves_the_tangential_components():
    """The property that makes sliding work."""
    out = project([0.3, 0.1, -0.5], TABLE_PLANE)
    assert out[0] == pytest.approx(0.3)
    assert out[1] == pytest.approx(0.1)


def test_lateral_motion_under_the_table_moves_the_target_laterally():
    # This is the regression: before projecting, the leash turned 10 cm of
    # sideways hand motion into almost no sideways target motion.
    a = project([0.30, 0.0, -0.40], TABLE_PLANE)
    b = project([0.40, 0.0, -0.40], TABLE_PLANE)
    assert b[0] - a[0] == pytest.approx(0.10)
    assert a[2] == b[2] == pytest.approx(-0.005)


def test_projection_never_moves_further_than_the_shortfall():
    for depth in (0.001, 0.05, 0.5, 2.0):
        p = np.array([0.3, 0.1, -0.005 - depth])
        assert np.linalg.norm(project(p, TABLE_PLANE) - p) == pytest.approx(depth)


def test_no_planes_is_a_no_op():
    p = [1.0, 2.0, 3.0]
    assert np.allclose(project(p, []), p)


def test_two_orthogonal_planes_both_end_up_satisfied():
    planes = [(np.array([0.0, 0.0, 1.0]), 0.0),
              (np.array([1.0, 0.0, 0.0]), 0.2)]
    out = project([-0.5, 0.0, -0.5], planes)
    assert out[2] >= -1e-9
    assert out[0] >= 0.2 - 1e-9


def test_projection_is_idempotent():
    once = project([0.3, 0.1, -0.5], TABLE_PLANE)
    assert np.allclose(project(once, TABLE_PLANE), once)


def test_violations_reports_depth_worst_first():
    planes = [(np.array([0.0, 0.0, 1.0]), 0.0),
              (np.array([1.0, 0.0, 0.0]), 0.5)]
    out = violations([0.1, 0.0, -0.02], planes)
    assert [i for i, _ in out] == [1, 0]           # 0.4 deep, then 0.02
    assert out[0][1] == pytest.approx(0.4)
    assert out[1][1] == pytest.approx(0.02)


def test_a_point_on_the_surface_is_not_a_violation():
    assert violations([0.3, 0.0, -0.005], TABLE_PLANE) == []


# ---- wall clearance -------------------------------------------------------
# Found by driving it: with the wall exactly on the box face, tcp_link sits at
# the surface and the GRIPPER FINGERS, which reach past that point, are inside
# the box. The checker then refuses every command and the arm stops dead with
# collision_scale 0.0, unable to recover -- "gripper in the table top and not
# moving". The clearance is the tool's reach past tcp_link.

def test_clearance_lifts_the_wall_off_the_face():
    box = cfg(wall_clearance=0.02).boxes[0]
    top = box.center[2] + box.size[2] / 2
    assert box.wall_plane().offset == pytest.approx(top + 0.02)


def test_zero_clearance_is_the_face_itself():
    assert cfg().boxes[0].wall_plane().offset == pytest.approx(-0.005)


def test_clearance_applies_to_every_face_in_its_own_direction():
    # The offset is on the half-space constant, so a positive clearance always
    # shrinks the allowed region regardless of which way the face points.
    for face in ('+x', '-x', '+y', '-y', '+z', '-z'):
        plain = cfg(wall=face).boxes[0].wall_plane()
        lifted = cfg(wall=face, wall_clearance=0.03).boxes[0].wall_plane()
        assert lifted.offset == pytest.approx(plain.offset + 0.03)
        # A point exactly on the bare face is now outside the allowed region.
        on_face = plain.normal * plain.offset
        assert float(lifted.normal @ on_face) < lifted.offset


def test_clearance_still_cannot_drift_from_the_box():
    for zmax in (-0.005, 0.0, 0.4):
        box = cfg(bounds=[-0.4, 0.4, -0.4, 0.4, zmax - 0.02, zmax],
                  wall_clearance=0.025).boxes[0]
        top = box.center[2] + box.size[2] / 2
        assert box.wall_plane().offset == pytest.approx(top + 0.025)


# ---- bounded walls --------------------------------------------------------
# An infinite half-space is the honest model for a floor and the wrong one for
# a table: a table you can reach around. Unbounded, a table holds the tool at
# table height across the entire workspace, including well past its edge.

def test_a_box_wall_is_bounded_by_the_box_footprint():
    plane = cfg(wall_bounded=True).boxes[0].wall_plane()
    assert plane.extent is not None
    assert np.allclose(plane.extent, [-0.4, 0.4, -0.4, 0.4])


def test_the_footprint_axes_are_the_ones_perpendicular_to_the_normal():
    # axes are a property of the normal, bounded or not.
    assert cfg(wall='+z').boxes[0].wall_plane().axes == (0, 1)   # x, y
    assert cfg(wall='+x').boxes[0].wall_plane().axes == (1, 2)   # y, z
    assert cfg(wall='-y').boxes[0].wall_plane().axes == (0, 2)   # x, z


def test_wall_margin_widens_the_footprint():
    plane = cfg(wall_bounded=True, wall_margin=0.05).boxes[0].wall_plane()
    assert np.allclose(plane.extent, [-0.45, 0.45, -0.45, 0.45])


def test_walls_are_infinite_by_default():
    # Bounded walls are opt-in: a vanishing constraint steps the solver's
    # optimum 218 mm at the footprint edge on this arm. See the module
    # docstring on Plane.
    assert cfg().boxes[0].wall_plane().extent is None
    assert cfg(wall_bounded=True).boxes[0].wall_plane().extent is not None


def test_applies_at_is_true_over_the_footprint_and_false_beyond():
    plane = cfg(wall_bounded=True).boxes[0].wall_plane()
    assert plane.applies_at([0.0, 0.0, -5.0])
    assert plane.applies_at([0.4, -0.4, 0.0])        # exactly on the edge
    assert not plane.applies_at([0.45, 0.0, 0.0])    # reached past the side


def test_projection_ignores_a_wall_the_point_has_reached_past():
    planes = [cfg(wall_bounded=True).boxes[0].wall_plane()]
    over = project([0.0, 0.0, -0.5], planes)
    assert over[2] == pytest.approx(-0.005), 'over the table: held up'
    past = project([0.6, 0.0, -0.5], planes)
    assert past[2] == pytest.approx(-0.5), 'past the edge: free to descend'


def test_violations_ignore_a_wall_the_point_has_reached_past():
    planes = [cfg(wall_bounded=True).boxes[0].wall_plane()]
    assert violations([0.0, 0.0, -0.5], planes)
    assert violations([0.6, 0.0, -0.5], planes) == []


def test_an_unbounded_wall_still_applies_everywhere():
    planes = [cfg().boxes[0].wall_plane()]
    assert project([5.0, 5.0, -0.5], planes)[2] == pytest.approx(-0.005)


def test_a_standalone_plane_can_declare_its_own_rectangle():
    scene = SceneConfig.from_dict({'planes': [
        {'name': 'shelf', 'frame': 'base_link', 'normal': [0, 0, 1],
         'offset': 0.2, 'extent': [0.1, 0.5, -0.2, 0.2]}]})
    plane = scene.planes[0]
    assert np.allclose(plane.extent, [0.1, 0.5, -0.2, 0.2])
    assert plane.applies_at([0.3, 0.0, 0.0])
    assert not plane.applies_at([0.05, 0.0, 0.0])


def test_a_tilted_plane_cannot_carry_a_rectangle():
    # "The two perpendicular axes" is a choice, not a fact, for a tilted
    # normal; guessing would bound the wall along axes nobody asked for.
    with pytest.raises(SceneConfigError, match='axis-aligned'):
        SceneConfig.from_dict({'planes': [
            {'name': 'ramp', 'normal': [1, 0, 1], 'offset': 0.0,
             'extent': [0, 1, 0, 1]}]})


def test_a_wrong_length_extent_is_rejected():
    with pytest.raises(SceneConfigError, match='a_min'):
        SceneConfig.from_dict({'planes': [
            {'name': 'p', 'normal': [0, 0, 1], 'offset': 0.0,
             'extent': [0, 1, 0]}]})


def test_a_translated_frame_shifts_the_footprint():
    plane = cfg(wall_bounded=True).boxes[0].wall_plane()
    moved = plane.transformed(np.eye(3), [0.1, -0.2, 0.05])
    assert np.allclose(moved.extent, [-0.3, 0.5, -0.6, 0.2])
    assert moved.offset == pytest.approx(-0.005 + 0.05)


def test_a_rotated_frame_with_a_footprint_is_refused():
    from scipy.spatial.transform import Rotation
    plane = cfg(wall_bounded=True).boxes[0].wall_plane()
    R = Rotation.from_euler('z', 30, degrees=True).as_matrix()
    with pytest.raises(SceneConfigError, match='rotated'):
        plane.transformed(R, [0.0, 0.0, 0.0])


def test_a_rotated_frame_without_a_footprint_is_fine():
    from scipy.spatial.transform import Rotation
    plane = cfg().boxes[0].wall_plane()
    R = Rotation.from_euler('z', 30, degrees=True).as_matrix()
    assert plane.transformed(R, [0.0, 0.0, 0.0]).extent is None
