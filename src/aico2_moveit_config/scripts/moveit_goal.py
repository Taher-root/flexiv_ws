#!/usr/bin/env python3
"""Plan and execute a MoveIt goal from the command line. No RViz, no display.

RViz needs an X display, which an SSH session into the robot usually does not
have. This drives move_group's /move_action directly instead, so the MoveIt
layer can be tested headless:

    source /opt/ros/jazzy/setup.bash
    source install/setup.bash

    # what named states exist
    python3 src/aico2_moveit_config/scripts/moveit_goal.py --list

    # plan only -- proves planning works without moving anything
    python3 src/aico2_moveit_config/scripts/moveit_goal.py --named ready --plan-only

    # plan and execute
    python3 src/aico2_moveit_config/scripts/moveit_goal.py --named ready --yes-move

    # a relative nudge, like check_arm_ros.py but through MoveIt
    python3 src/aico2_moveit_config/scripts/moveit_goal.py \
        --joint 4 --degrees 15 --yes-move

Targets are joint-space (JointConstraint), not poses, deliberately: it isolates
planning and execution from IK, which on a 7-DoF arm with the KDL solver is the
other thing most likely to fail. Once this works, a failing pose goal in RViz is
an IK problem; if this fails, IK was never the issue.

--plan-only is the useful middle rung. It exercises the SRDF, the planning
pipeline, joint limits and collision checking, and touches no hardware. Run it
before anything with --yes-move.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

_ACTION = "/move_action"
# moveit_msgs/MoveItErrorCodes, the values worth naming.
_ERROR_CODES = {
    1: "SUCCESS",
    -1: "FAILURE",
    -2: "PLANNING_FAILED",
    -3: "INVALID_MOTION_PLAN",
    -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -5: "CONTROL_FAILED",
    -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    -7: "TIMED_OUT",
    -8: "PREEMPTED",
    -10: "START_STATE_IN_COLLISION",
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -12: "GOAL_IN_COLLISION",
    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -14: "GOAL_CONSTRAINTS_VIOLATED",
    -15: "INVALID_GROUP_NAME",
    -16: "INVALID_GOAL_CONSTRAINTS",
    -17: "INVALID_ROBOT_STATE",
    -18: "INVALID_LINK_NAME",
    -19: "INVALID_OBJECT_NAME",
    -21: "FRAME_TRANSFORM_FAILURE",
    -22: "COLLISION_CHECKING_UNAVAILABLE",
    -23: "ROBOT_STATE_STALE",
    -24: "SENSOR_INFO_STALE",
    -25: "COMMUNICATION_FAILURE",
    -31: "NO_IK_SOLUTION",
}


def load_srdf_states(group):
    """Named states for a group, read from the installed SRDF."""
    srdf = os.path.join(get_package_share_directory("aico2_moveit_config"),
                        "config", "aico2.srdf")
    root = ET.parse(srdf).getroot()
    states = {}
    for gs in root.findall("group_state"):
        if gs.get("group") != group:
            continue
        states[gs.get("name")] = {j.get("name"): float(j.get("value"))
                                  for j in gs.findall("joint")}
    return states, srdf


class GoalSender(Node):
    def __init__(self):
        super().__init__("moveit_goal")
        self._client = ActionClient(self, MoveGroup, _ACTION)
        self._latest = {}
        self.create_subscription(JointState, "/joint_states", self._on_js,
                                 qos_profile_sensor_data)

    def _on_js(self, msg):
        for name, position in zip(msg.name, msg.position):
            self._latest[name] = position

    def wait_for_joint_states(self, names, timeout=5.0):
        deadline = self.get_clock().now().nanoseconds * 1e-9 + timeout
        while self.get_clock().now().nanoseconds * 1e-9 < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(n in self._latest for n in names):
                return {n: self._latest[n] for n in names}
        missing = [n for n in names if n not in self._latest]
        raise TimeoutError(f"no /joint_states for {missing} within {timeout:.0f}s")

    def send(self, group, targets, plan_only, args):
        if not self._client.wait_for_server(timeout_sec=args.timeout):
            raise RuntimeError(
                f"no move_group action server at {_ACTION} within "
                f"{args.timeout:.0f}s — is move_group.launch.py running?")

        constraints = Constraints()
        for name, position in targets.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(position)
            jc.tolerance_above = args.tolerance
            jc.tolerance_below = args.tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.goal_constraints = [constraints]
        goal.request.num_planning_attempts = args.attempts
        goal.request.allowed_planning_time = args.planning_time
        goal.request.max_velocity_scaling_factor = args.velocity_scaling
        goal.request.max_acceleration_scaling_factor = args.acceleration_scaling
        goal.planning_options.plan_only = plan_only
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        print(f"\ngroup {group}, {'PLAN ONLY' if plan_only else 'PLAN + EXECUTE'}")
        for name, position in targets.items():
            current = self._latest.get(name)
            now = f"{math.degrees(current):8.2f}°" if current is not None else "   ?   "
            print(f"  {name:14s} {now} -> {math.degrees(position):8.2f}°")

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future,
                                         timeout_sec=args.timeout)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("move_group rejected the goal")
        print("goal accepted, planning...")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future,
            timeout_sec=args.planning_time + args.timeout + 60.0)
        if result_future.result() is None:
            raise TimeoutError("no result from move_group")

        result = result_future.result().result
        code = result.error_code.val
        name = _ERROR_CODES.get(code, f"unknown({code})")
        planned = len(result.planned_trajectory.joint_trajectory.points)
        print(f"\nerror_code {code} ({name})")
        print(f"planned trajectory: {planned} points, "
              f"planning time {result.planning_time:.3f}s")
        if code == 1 and not plan_only:
            reached = self.wait_for_joint_states(list(targets), timeout=3.0)
            print("final positions:")
            for jname, position in targets.items():
                err = math.degrees(reached[jname] - position)
                print(f"  {jname:14s} {math.degrees(reached[jname]):8.2f}°  "
                      f"error {err:+6.2f}°")
        elif code == -31:
            print("NO_IK_SOLUTION on a joint-space goal is odd — check that the "
                  "group name matches the SRDF.")
        elif code == -15:
            print(f"INVALID_GROUP_NAME: {group!r} is not a group in the SRDF.")
        elif code == -5:
            print("CONTROL_FAILED: planning worked, execution did not. The "
                  "controller name in moveit_controllers.yaml must match the "
                  "driver's namespace — check `ros2 action list | grep follow`.")
        return 0 if code == 1 else 1


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--group", default="left_arm")
    ap.add_argument("--list", action="store_true",
                    help="list the group's named states and exit")
    ap.add_argument("--named", help="SRDF named state to move to, e.g. ready")
    ap.add_argument("--joint", type=int,
                    help="arm joint 1-7 for a relative move")
    ap.add_argument("--degrees", type=float, default=15.0,
                    help="relative move size with --joint")
    ap.add_argument("--plan-only", action="store_true",
                    help="plan without executing (no motion, no --yes-move)")
    ap.add_argument("--yes-move", action="store_true",
                    help="required to execute")
    ap.add_argument("--tolerance", type=float, default=0.01,
                    help="joint constraint tolerance, radians")
    ap.add_argument("--attempts", type=int, default=10)
    ap.add_argument("--planning-time", type=float, default=5.0)
    ap.add_argument("--velocity-scaling", type=float, default=0.2)
    ap.add_argument("--acceleration-scaling", type=float, default=0.2)
    ap.add_argument("--timeout", type=float, default=10.0)
    args = ap.parse_args()

    states, srdf_path = load_srdf_states(args.group)
    if args.list:
        print(f"SRDF: {srdf_path}")
        print(f"named states for group {args.group!r}:")
        for name, joints in states.items():
            values = " ".join(f"{math.degrees(v):.1f}" for v in joints.values())
            print(f"  {name:10s} {values} (deg)")
        return 0

    if not args.named and args.joint is None:
        raise SystemExit("give --named <state> or --joint <n> (or --list)")
    if args.named and args.joint is not None:
        raise SystemExit("--named and --joint are mutually exclusive")
    if not args.plan_only and not args.yes_move:
        raise SystemExit(
            "executing moves the arm. Add --yes-move, or --plan-only to plan\n"
            "without touching the hardware (do that first).")
    if args.named and args.named not in states:
        raise SystemExit(
            f"{args.named!r} is not a named state for {args.group!r}; "
            f"have {sorted(states)}")

    prefix = "Right" if "right" in args.group else "Left"
    joint_names = [f"{prefix}_joint{i}" for i in range(1, 8)]

    rclpy.init()
    node = GoalSender()
    try:
        if args.named:
            targets = states[args.named]
        else:
            if not 1 <= args.joint <= 7:
                raise SystemExit("--joint must be 1..7")
            current = node.wait_for_joint_states(joint_names)
            name = joint_names[args.joint - 1]
            targets = {name: current[name] + math.radians(args.degrees)}
        # Populate current positions for the printout even on a named goal.
        try:
            node.wait_for_joint_states(joint_names, timeout=2.0)
        except TimeoutError:
            pass
        return node.send(args.group, targets, args.plan_only, args)
    except (RuntimeError, TimeoutError) as exc:
        print(f"\nFAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
