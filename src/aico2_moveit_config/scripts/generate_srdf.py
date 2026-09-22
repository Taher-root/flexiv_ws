#!/usr/bin/env python3
"""Regenerate config/aico2.srdf from the URDF's kinematic tree.

Run this after changing AICO2-Rizon4.urdf so the collision matrix keeps up:

    python3 scripts/generate_srdf.py \
        ../flexiv_amr_description/urdf/AICO2-Rizon4.urdf > config/aico2.srdf

What it derives:

  Welded    links in the same rigid body, i.e. connected through a chain of
            fixed joints. They cannot move relative to each other.
  Adjacent  links in rigid bodies joined by one actuated joint. They pivot
            against each other by construction.

The rigid-body step matters. Raw parent/child adjacency is not enough here:
AGV_Pitch -> Left_link0 is a FIXED joint, so the torso plate and both arm base
stubs are one body, and Left_link1 is adjacent to all of it across
Left_joint1. Without collapsing fixed joints first, AGV_Pitch and Left_link1
look two hops apart and stay collision-checked -- which is exactly what made
move_group refuse to plan:

    PlanningRequestAdapter 'CheckStartStateCollision' failed, because
    '2 contact(s) detected : AGV_Pitch - Left_link1, AGV_Pitch - Right_link1'

One consequence worth knowing: body {AGV_Pitch, Left_link0, Right_link0} is
adjacent to Left_link1 AND Right_link1, so Left_link1 vs Right_link0 is
disabled too. Both link0s are short stubs on the plate whose geometry dominates
the pair, so this is a fair trade, but it does mean a genuine contact between
one arm's first link and the other arm's base mount would not be caught.

Still NOT derivable from the tree: the Setup Assistant's "Never" (pairs that
never collide anywhere reachable) and the remaining "Default" pairs. Those need
geometric sampling against the meshes. Planning works without them, with more
collision checks per state and conservative cross-arm checking.
"""
from __future__ import annotations

import argparse
import math
import re
import sys

ARM_SIDES = ("Left", "Right")
# Matches mock_idle_joint_positions in {left,right}_arm.yaml.
HOME = [0.0, -0.7145, 0.0, 1.6559, 0.0, 0.0, 0.0]
# home has joint6=0, a wrist singularity where MoveIt Servo halts.
READY = [0.0, -0.7145, 0.0, 1.6559, 0.0, math.pi / 2, 0.0]


def parse_robot_name(urdf_text):
    """The URDF's robot name. The SRDF must use the same one.

    Getting this wrong is not fatal but MoveIt says so on every start:
        Error: Semantic description is not specified for the same robot as
        the URDF   at line 681 in ./src/model.cpp
    """
    m = re.search(r'<robot\s+name="([^"]+)"', urdf_text)
    if not m:
        raise SystemExit("could not find <robot name=...> in the URDF")
    return m.group(1)


def parse_joints(urdf_text):
    """(name, type, parent_link, child_link) for every joint in the URDF."""
    out = []
    for m in re.finditer(
        r'<joint name="([^"]+)"\s+type="([^"]+)"\s*>(.*?)</joint>',
        urdf_text, re.S,
    ):
        name, jtype, body = m.group(1), m.group(2), m.group(3)
        parent = re.search(r'<parent link="([^"]+)"', body)
        child = re.search(r'<child link="([^"]+)"', body)
        if parent and child:
            out.append((name, jtype, parent.group(1), child.group(1)))
    return out


def rigid_bodies(joints):
    """Map each link to its rigid body, collapsing fixed joints.

    Returns {representative_link: sorted[member links]} and {link: representative}.
    """
    parent = {}

    def find(link):
        parent.setdefault(link, link)
        while parent[link] != link:
            parent[link] = parent[parent[link]]
            link = parent[link]
        return link

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    links = set()
    for _, _, p, c in joints:
        links.update((p, c))
    for link in links:
        find(link)
    for _, jtype, p, c in joints:
        if jtype == "fixed":
            union(p, c)

    bodies = {}
    for link in links:
        bodies.setdefault(find(link), []).append(link)
    return {k: sorted(v) for k, v in bodies.items()}, {l: find(l) for l in links}


def collision_pairs(joints):
    pairs = {}

    def add(a, b, reason):
        if a != b:
            pairs.setdefault(tuple(sorted((a, b))), reason)

    bodies, body_of = rigid_bodies(joints)

    # Within one rigid body nothing can move, so nothing can newly collide.
    for members in bodies.values():
        for i, a in enumerate(members):
            for b in members[i + 1:]:
                add(a, b, "Welded")

    # Across one actuated joint, the two bodies pivot against each other.
    for _, jtype, p, c in joints:
        if jtype == "fixed":
            continue
        for a in bodies[body_of[p]]:
            for b in bodies[body_of[c]]:
                add(a, b, "Adjacent")
    return pairs


def emit(joints, robot_name, out=sys.stdout):
    pairs = collision_pairs(joints)
    arm_joints = {side: [f"{side}_joint{i}" for i in range(1, 8)]
                  for side in ARM_SIDES}
    w = out.write

    w('<?xml version="1.0" encoding="UTF-8"?>\n')
    w('<!--\n')
    w(f'  Semantic description for {robot_name} '
      '(two Rizon 4 arms on a 2-DoF waist).\n\n')
    w('  GENERATED from flexiv_amr_description/urdf/AICO2-Rizon4.urdf by\n')
    w('  scripts/generate_srdf.py. The disable_collisions entries below cover\n')
    w('  only the pairs derivable from the kinematic tree, after collapsing\n')
    w('  fixed joints into rigid bodies:\n')
    w('    Welded   - same rigid body, cannot move relative to each other\n')
    w('    Adjacent - rigid bodies joined by one actuated joint\n\n')
    w('  NOT covered: pairs that never collide across the reachable workspace,\n')
    w('  and the remaining pairs that already collide in some pose. Those need\n')
    w('  geometric sampling against the meshes, i.e. a pass with the MoveIt\n')
    w('  Setup Assistant on this URDF. Without them planning still works but\n')
    w('  is slower and cross-arm checking stays conservative.\n')
    w('  See docs/moveit_status.md.\n')
    w('-->\n')
    w(f'<robot name="{robot_name}">\n')
    for side in ARM_SIDES:
        w(f'  <group name="{side.lower()}_arm">\n')
        w(f'    <chain base_link="{side}_link0" tip_link="{side}_flange"/>\n')
        w('  </group>\n')
    w('  <group name="both_arms">\n')
    w('    <group name="left_arm"/>\n')
    w('    <group name="right_arm"/>\n')
    w('  </group>\n\n')
    w('  <!-- home matches mock_idle_joint_positions in left_arm.yaml. joint6=0\n')
    w('       there is a wrist singularity where MoveIt Servo halts, so `ready`\n')
    w('       moves joint6 to +pi/2: plan to `ready` before engaging teleop. -->\n')
    for side in ARM_SIDES:
        for state_name, values in (("home", HOME), ("ready", READY)):
            w(f'  <group_state name="{state_name}" group="{side.lower()}_arm">\n')
            for jname, value in zip(arm_joints[side], values):
                w(f'    <joint name="{jname}" value="{value:.4f}"/>\n')
            w('  </group_state>\n')
    w('\n')
    w('  <!-- The waist carries both arm bases (Left_Joint0/Right_Joint0 are\n')
    w('       fixed to AGV_Pitch) but nothing in ROS commands it, so it is in no\n')
    w('       planning group: MoveIt sees it as a moving base via /joint_states. -->\n')
    w('  <passive_joint name="AGV_Joint1"/>\n')
    w('  <passive_joint name="AGV_Joint2"/>\n\n')
    for (a, b), reason in sorted(pairs.items()):
        w(f'  <disable_collisions link1="{a}" link2="{b}" reason="{reason}"/>\n')
    w('</robot>\n')
    return len(pairs)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("urdf", help="path to AICO2-Rizon4.urdf")
    args = ap.parse_args()
    urdf_text = open(args.urdf, encoding="utf-8").read()
    robot_name = parse_robot_name(urdf_text)
    joints = parse_joints(urdf_text)
    if not joints:
        raise SystemExit(f"no joints parsed from {args.urdf}")
    count = emit(joints, robot_name)
    print(f"robot {robot_name!r}: {len(joints)} joints, "
          f"{count} collision pairs disabled", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
