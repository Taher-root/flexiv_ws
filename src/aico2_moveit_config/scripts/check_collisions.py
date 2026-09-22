#!/usr/bin/env python3
"""Ask move_group which links it thinks are colliding, and why planning refuses.

When CheckStartStateCollision aborts a plan, move_group's log names the contact
pairs but nothing else does -- error_code FAILURE says only that an adapter
refused. This calls move_group's /check_state_validity service directly, so the
contacts come back as data:

    source /opt/ros/jazzy/setup.bash
    python3 src/aico2_moveit_config/scripts/check_collisions.py

    # check where the arm is now, and where a named state would put it
    python3 .../check_collisions.py --named ready

    # check a relative move BEFORE executing it -- same arguments as
    # moveit_goal.py, so the answer applies to the move you are about to make
    python3 .../check_collisions.py --joint 4 --degrees -15

    # ready-to-paste SRDF lines for whatever it found
    python3 .../check_collisions.py --emit-srdf

This is the tool to reach for instead of RViz when the question is "which pair",
and it needs no display. RViz is better for "why does that pair touch" -- run it
on a machine with a screen, on the same ROS_DOMAIN_ID.

A contact reported here is one of two things:

  - a pair missing from disable_collisions, i.e. links that are always in
    contact by construction. Add them (--emit-srdf prints the lines) and it
    goes away. This is the common case, and what the MoveIt Setup Assistant's
    "Never" and "Default" categories would have covered.
  - a genuine self-collision, i.e. the arm really is folded into itself or the
    torso. Then the fix is to move the arm, not to edit the SRDF. `depth` tells
    them apart: a fraction of a millimetre is geometry touching at a joint,
    while centimetres means real interpenetration.

If it reports the arm is valid but planning still fails, move_group is running
against a stale SRDF -- it reads it once at startup. Restart it.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

_SERVICE = "/check_state_validity"


def srdf_path():
    return os.path.join(get_package_share_directory("aico2_moveit_config"),
                        "config", "aico2.srdf")


def load_named_state(group, name):
    root = ET.parse(srdf_path()).getroot()
    for gs in root.findall("group_state"):
        if gs.get("group") == group and gs.get("name") == name:
            return {j.get("name"): float(j.get("value"))
                    for j in gs.findall("joint")}
    return None


def already_disabled():
    root = ET.parse(srdf_path()).getroot()
    return {tuple(sorted((d.get("link1"), d.get("link2"))))
            for d in root.iter("disable_collisions")}


class CollisionChecker(Node):
    def __init__(self):
        super().__init__("check_collisions")
        self._client = self.create_client(GetStateValidity, _SERVICE)
        self._names, self._positions = [], []
        self.create_subscription(JointState, "/joint_states", self._on_js,
                                 qos_profile_sensor_data)

    def _on_js(self, msg):
        # Accumulate across publishers: with the merger running there is more
        # than one, and a single message may carry only part of the robot.
        latest = dict(zip(self._names, self._positions))
        latest.update(dict(zip(msg.name, msg.position)))
        self._names = list(latest)
        self._positions = [latest[n] for n in self._names]

    def wait_for_state(self, timeout=5.0):
        deadline = self.get_clock().now().nanoseconds * 1e-9 + timeout
        while self.get_clock().now().nanoseconds * 1e-9 < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if len(self._names) >= 7:
                return dict(zip(self._names, self._positions))
        raise TimeoutError(
            f"only {len(self._names)} joints on /joint_states within "
            f"{timeout:.0f}s — are the drivers active?")

    def check(self, group, joint_values, label, timeout=5.0):
        if not self._client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(
                f"no {_SERVICE} service within {timeout:.0f}s — is "
                f"move_group.launch.py running?")
        state = RobotState()
        state.joint_state = JointState()
        state.joint_state.name = list(joint_values)
        state.joint_state.position = [float(v) for v in joint_values.values()]
        state.is_diff = False

        request = GetStateValidity.Request()
        request.robot_state = state
        request.group_name = group

        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            raise TimeoutError(f"{_SERVICE} did not answer")
        response = future.result()

        print()
        print("=" * 72)
        print(f"{label} — group {group}")
        print("=" * 72)
        for name in sorted(joint_values):
            print(f"  {name:14s} {math.degrees(joint_values[name]):8.2f}°")
        print(f"\nvalid: {response.valid}")

        contacts = list(response.contacts)
        if not contacts:
            print("no contacts reported")
            return response.valid, []

        disabled = already_disabled()
        pairs = []
        print(f"\n{len(contacts)} contact(s):")
        for contact in contacts:
            a, b = contact.contact_body_1, contact.contact_body_2
            key = tuple(sorted((a, b)))
            depth_mm = contact.depth * 1000.0
            note = "already in disable_collisions(!)" if key in disabled else ""
            verdict = ("touching" if abs(depth_mm) < 1.0
                       else "INTERPENETRATING")
            print(f"  {a:22s} <-> {b:22s} depth {depth_mm:7.2f} mm  "
                  f"{verdict} {note}")
            if key not in disabled:
                pairs.append(key)
        return response.valid, pairs


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--group", default="left_arm")
    ap.add_argument("--named",
                    help="also check this SRDF named state (the goal), not just "
                         "the current pose")
    ap.add_argument("--joint", type=int,
                    help="also check the target of a relative move on this arm "
                         "joint (1-7), the way moveit_goal.py would command it")
    ap.add_argument("--degrees", type=float, default=-15.0,
                    help="size of that relative move")
    ap.add_argument("--emit-srdf", action="store_true",
                    help="print disable_collisions lines for pairs found")
    ap.add_argument("--timeout", type=float, default=5.0)
    args = ap.parse_args()

    rclpy.init()
    node = CollisionChecker()
    try:
        current = node.wait_for_state(args.timeout)
        valid, pairs = node.check(args.group, current, "CURRENT POSE",
                                  args.timeout)

        if args.joint is not None:
            if not 1 <= args.joint <= 7:
                raise SystemExit("--joint must be 1..7")
            prefix = "Right" if "right" in args.group else "Left"
            name = f"{prefix}_joint{args.joint}"
            if name not in current:
                raise SystemExit(f"{name} is not in /joint_states")
            moved = dict(current)
            moved[name] = current[name] + math.radians(args.degrees)
            move_valid, move_pairs = node.check(
                args.group, moved,
                f"TARGET after {args.degrees:+.1f}° on {name}", args.timeout)
            valid = valid and move_valid
            pairs = pairs + [p for p in move_pairs if p not in pairs]

        if args.named:
            target = load_named_state(args.group, args.named)
            if target is None:
                raise SystemExit(
                    f"{args.named!r} is not a named state for {args.group!r}")
            # Named states only name the group's joints; the rest of the robot
            # (the waist, the other arm) has to keep its current values or the
            # check runs against a half-defined robot.
            full = dict(current)
            full.update(target)
            goal_valid, goal_pairs = node.check(
                args.group, full, f"GOAL STATE {args.named!r}", args.timeout)
            valid = valid and goal_valid
            pairs = pairs + [p for p in goal_pairs if p not in pairs]

        print()
        if valid and not pairs:
            print("Every state checked is collision-free as far as move_group")
            print("is concerned. Note that MoveIt checks this itself on every")
            print("plan, so a move that would collide is refused at planning")
            print("time rather than executed — planning success is already the")
            print("collision guarantee. This script is for when it refuses and")
            print("you want to know which pair.")
            print()
            print("If planning still fails, move_group is running")
            print("against a stale SRDF — it reads it once at startup, so")
            print("restart it after regenerating aico2.srdf.")
            return 0

        if pairs:
            print(f"{len(pairs)} pair(s) not yet in disable_collisions.")
            print("Check the depth column first: under a millimetre is geometry")
            print("touching at a joint, which belongs in the SRDF. Centimetres")
            print("is real interpenetration — move the arm instead.")
            if args.emit_srdf:
                print("\nLines for config/aico2.srdf (before </robot>):")
                for a, b in pairs:
                    print(f'  <disable_collisions link1="{a}" link2="{b}" '
                          f'reason="Default"/>')
            else:
                print("\nRe-run with --emit-srdf for pasteable SRDF lines.")
        return 1
    except (RuntimeError, TimeoutError) as exc:
        print(f"\nFAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
