#!/usr/bin/env python3
"""Regenerate config/aico2.srdf from the URDF's kinematic tree.

Run this after changing AICO2-Rizon4.urdf so the collision matrix keeps up:

    python3 scripts/generate_srdf.py \
        ../flexiv_amr_description/urdf/AICO2-Rizon4.urdf > config/aico2.srdf

What it derives, and what it cannot:

  Adjacent  parent/child links across a joint. They touch by construction, so
            self-collision checking between them is always wasted work.
  Welded    links rigidly connected through a chain of fixed joints. They
            cannot move relative to each other, so the same applies.

Everything else the MoveIt Setup Assistant would emit -- "Never" (pairs that
never collide anywhere in the reachable workspace) and "Default" (pairs already
colliding in the zero pose) -- needs geometric sampling against the meshes and
is not derivable from the tree. Planning works without them, just with more
collision checks per state, and cross-arm checks stay conservative.
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


def welded_components(joints):
    """Connected components of links joined only by fixed joints."""
    adjacency = {}
    for _, jtype, parent, child in joints:
        if jtype != "fixed":
            continue
        adjacency.setdefault(parent, set()).add(child)
        adjacency.setdefault(child, set()).add(parent)

    components, seen = [], set()
    for link in adjacency:
        if link in seen:
            continue
        stack, component = [link], set()
        while stack:
            current = stack.pop()
            if current in component:
                continue
            component.add(current)
            seen.add(current)
            stack.extend(adjacency.get(current, ()))
        if len(component) > 1:
            components.append(sorted(component))
    return components


def collision_pairs(joints):
    pairs = {}

    def add(a, b, reason):
        if a != b:
            pairs.setdefault(tuple(sorted((a, b))), reason)

    for _, _, parent, child in joints:
        add(parent, child, "Adjacent")
    for component in welded_components(joints):
        for i, a in enumerate(component):
            for b in component[i + 1:]:
                add(a, b, "Welded")
    return pairs


def emit(joints, out=sys.stdout):
    pairs = collision_pairs(joints)
    arm_joints = {side: [f"{side}_joint{i}" for i in range(1, 8)]
                  for side in ARM_SIDES}
    w = out.write

    w('<?xml version="1.0" encoding="UTF-8"?>\n')
    w('<!--\n')
    w('  Semantic description for AICO2 (two Rizon 4 arms on a 2-DoF waist).\n\n')
    w('  GENERATED from flexiv_amr_description/urdf/AICO2-Rizon4.urdf by\n')
    w('  scripts/generate_srdf.py. The disable_collisions entries below cover\n')
    w('  only the pairs derivable from the kinematic tree:\n')
    w('    Adjacent - parent/child across a joint, always touching\n')
    w('    Welded   - rigidly connected through fixed joints\n\n')
    w('  NOT covered: pairs that never collide across the reachable workspace,\n')
    w('  or that already collide in the zero pose. Those need geometric\n')
    w('  sampling against the meshes, i.e. a pass with the MoveIt Setup\n')
    w('  Assistant on this URDF. Without them planning still works but is\n')
    w('  slower, and self-collision checks between the two arms and the AMR\n')
    w('  body stay conservative. See docs/moveit_status.md.\n')
    w('-->\n')
    w('<robot name="AICO2">\n')
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
    joints = parse_joints(open(args.urdf, encoding="utf-8").read())
    if not joints:
        raise SystemExit(f"no joints parsed from {args.urdf}")
    count = emit(joints)
    print(f"{len(joints)} joints, {count} collision pairs disabled",
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
