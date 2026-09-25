"""Unit tests for moveit_goal.py's --random sampler (no rclpy, no hardware).

moveit_goal.py imports rclpy, so the two pure functions are lifted out of the
source by AST and run against the real URDF. Worth testing rather than
eyeballing: a joint-limit violation is not a harmless planning failure on this
robot -- commanding Left_joint4 past its 159 deg limit faulted the arm earlier
in this project, and a sampler is exactly the kind of code that is right for
every pose you happen to try and wrong at the edges.
"""

import ast
import math
import os
import random
import textwrap
import xml.etree.ElementTree as ET
from pathlib import Path

_PKG = Path(__file__).resolve().parents[1]
_SRC = _PKG / "scripts" / "moveit_goal.py"
_DESCRIPTION = _PKG.parent / "flexiv_amr_description"

JOINTS = [f"Left_joint{i}" for i in range(1, 8)]
MARGIN_DEG = 5.0
RANGE_DEG = 25.0


def _load():
    text = _SRC.read_text()
    ns = {
        "math": math, "os": os, "random": random, "ET": ET,
        # The script resolves the URDF through the ament index; in a plain
        # pytest run nothing is installed, so point it at the source tree.
        "get_package_share_directory": lambda pkg: str(_DESCRIPTION),
    }
    for node in ast.parse(text).body:
        if (isinstance(node, ast.FunctionDef)
                and node.name in ("load_joint_limits", "random_targets")):
            exec(textwrap.dedent(ast.get_source_segment(text, node)), ns)
    return ns["load_joint_limits"], ns["random_targets"]


load_joint_limits, random_targets = _load()
LIMITS, _URDF = load_joint_limits(JOINTS)


def _sample(current, seed, range_deg=RANGE_DEG, margin_deg=MARGIN_DEG):
    return random_targets(current, LIMITS, range_deg, margin_deg,
                          random.Random(seed))


def _within_limits(targets, margin_deg=MARGIN_DEG):
    margin = math.radians(margin_deg)
    return all(LIMITS[n][0] + margin - 1e-9 <= v <= LIMITS[n][1] - margin + 1e-9
               for n, v in targets.items())


def test_urdf_limits_are_the_ones_we_think():
    # If the URDF changes these, the numbers quoted all over the docs are stale.
    assert [round(math.degrees(LIMITS[n][0])) for n in JOINTS] == \
        [-165, -135, -175, -112, -175, -85, -175]
    assert [round(math.degrees(LIMITS[n][1])) for n in JOINTS] == \
        [165, 135, 175, 159, 175, 265, 175]


def test_targets_every_joint_in_the_group():
    targets = _sample({n: 0.0 for n in JOINTS}, seed=1)
    assert sorted(targets) == sorted(JOINTS)


def test_stays_inside_limits_and_within_range():
    current = {n: 0.0 for n in JOINTS}
    current["Left_joint4"] = math.radians(90.0)
    targets = _sample(current, seed=1)
    assert _within_limits(targets)
    assert all(abs(v - current[n]) <= math.radians(RANGE_DEG) + 1e-9
               for n, v in targets.items())


def test_seed_makes_the_pose_reproducible():
    # --plan-only then --yes-move with the same seed must aim at the same pose,
    # or planning proves nothing about what gets executed.
    current = {n: 0.0 for n in JOINTS}
    assert _sample(current, seed=7) == _sample(current, seed=7)
    assert _sample(current, seed=7) != _sample(current, seed=8)


def test_starting_at_a_limit_does_not_sample_past_it():
    targets = _sample({n: LIMITS[n][1] for n in JOINTS}, seed=3)
    assert _within_limits(targets)


def test_starting_beyond_a_limit_clamps_instead_of_crashing():
    # rng.uniform(low, high) with low > high returns a value outside both, so
    # the empty-interval case has to be handled rather than sampled.
    beyond = {n: LIMITS[n][1] + math.radians(10.0) for n in JOINTS}
    targets = _sample(beyond, seed=4)
    assert _within_limits(targets)


def test_two_thousand_random_start_poses_never_leave_the_limits():
    rng = random.Random(99)
    for _ in range(2000):
        start = {n: rng.uniform(*LIMITS[n]) for n in JOINTS}
        assert _within_limits(random_targets(
            start, LIMITS, RANGE_DEG, MARGIN_DEG, rng))
