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

    # a random pose on all 7 joints -- plan it first, then run the same one
    python3 src/aico2_moveit_config/scripts/moveit_goal.py --random --plan-only
    python3 src/aico2_moveit_config/scripts/moveit_goal.py \
        --random --seed 12345 --yes-move

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
import random
import sys
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from aico2_msgs.msg import ArmStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

_ACTION = "/move_action"
# moveit_msgs/MoveItErrorCodes, transcribed from the ROS 2 message. Note that
# this numbering is NOT MoveIt 1's: there, -1 was FAILURE and the codes below it
# were shifted by one. Getting it wrong turns a clear diagnosis into a
# misleading one, so these are copied from the .msg rather than remembered.
_ERROR_CODES = {
    1: "SUCCESS",
    0: "UNDEFINED",
    99999: "FAILURE",
    -1: "PLANNING_FAILED",
    -2: "INVALID_MOTION_PLAN",
    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -4: "CONTROL_FAILED",
    -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    -6: "TIMED_OUT",
    -7: "PREEMPTED",
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
    -26: "START_STATE_INVALID",
    -27: "GOAL_STATE_INVALID",
    -28: "UNRECOGNIZED_GOAL_TYPE",
    -29: "CRASH",
    -30: "ABORT",
    -31: "NO_IK_SOLUTION",
}

# What a code usually means for this robot, printed alongside it.
_HINTS = {
    99999: "FAILURE is the generic abort. A planning request adapter usually "
           "rejected the request before planning began — move_group's own log "
           "names which one and why. 'CheckStartStateCollision failed' means "
           "the arm is in self-collision at its current pose according to the "
           "SRDF, which most often means a collision pair is missing from "
           "disable_collisions rather than a real contact.",
    -4: "CONTROL_FAILED: planning worked, execution did not. Check in this "
        "order:\n"
        "  1. Is the arm in fault? A faulted driver rejects every goal.\n"
        "       ros2 topic echo --once /left_arm/fault\n"
        "       ros2 service call /left_arm/clear_fault std_srvs/srv/Trigger\n"
        "     A minor fault usually follows a command that hit a joint limit.\n"
        "  2. Is teleop active? The driver refuses trajectories while it is.\n"
        "  3. Only then suspect the wiring: the controller name in\n"
        "     moveit_controllers.yaml must match the driver's namespace —\n"
        "     ros2 action list | grep follow",
    -10: "START_STATE_IN_COLLISION: same cause as above — usually a missing "
         "disable_collisions pair, not a real contact.",
    -15: "INVALID_GROUP_NAME: --group is not a group in the SRDF.",
    -31: "NO_IK_SOLUTION on a joint-space goal is odd; check the group name.",
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


def load_joint_limits(joint_names):
    """Position limits from the URDF -- the same file move_group plans against.

    Read by path rather than off /robot_description so this works with only
    move_group up: move_group.launch.py loads this exact file.
    """
    urdf = os.path.join(get_package_share_directory("flexiv_amr_description"),
                        "urdf", "AICO2-Rizon4.urdf")
    root = ET.parse(urdf).getroot()
    limits = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        if name not in joint_names:
            continue
        limit = joint.find("limit")
        if limit is not None:
            limits[name] = (float(limit.get("lower")), float(limit.get("upper")))
    missing = [n for n in joint_names if n not in limits]
    if missing:
        raise SystemExit(f"no position limits in {urdf} for {missing}")
    return limits, urdf


def random_targets(current, limits, range_deg, margin_deg, rng):
    """A random target per joint, within range_deg of now and inside the limits.

    Bounded relative to the current pose rather than sampled across the whole
    range. A uniform sample over seven joints puts the arm somewhere
    unpredictable, and "collision-free" here only means free of SELF-collision:
    the table, the fixtures and anything else in the cell are not in the
    planning scene, so MoveIt will happily plan straight through them.
    """
    span = math.radians(range_deg)
    margin = math.radians(margin_deg)
    targets = {}
    for name, position in current.items():
        lower, upper = limits[name]
        low = max(position - span, lower + margin)
        high = min(position + span, upper - margin)
        if low > high:
            # Already outside the margin band: aim for the nearest point in it
            # rather than sampling an empty interval.
            low = high = min(max(position, lower + margin), upper - margin)
        targets[name] = rng.uniform(low, high)
    return targets


class GoalSender(Node):
    def __init__(self, group):
        super().__init__("moveit_goal")
        self._client = ActionClient(self, MoveGroup, _ACTION)
        self._latest = {}
        self.create_subscription(JointState, "/joint_states", self._on_js,
                                 qos_profile_sensor_data)
        # MoveIt reports CONTROL_FAILED for anything the controller refuses or
        # fails to finish, which says nothing about why. The driver publishes
        # exactly that on ArmStatus, and the group name matches its namespace.
        self._status = None
        self.create_subscription(
            ArmStatus, f"/{group}/status", self._on_status, 10)

    def _on_js(self, msg):
        for name, position in zip(msg.name, msg.position):
            self._latest[name] = position

    def _on_status(self, msg):
        self._status = msg

    def status_line(self):
        if self._status is None:
            return None
        st = self._status
        return (f"driver: fault={st.fault} operational={st.operational} "
                f"enabled={st.enabled} mode={st.mode!r}"
                + (f" detail={st.detail!r}" if st.detail else ""))

    def print_status(self, prefix=""):
        line = self.status_line()
        if line:
            print(f"{prefix}{line}")
        return self._status

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
        # is_diff means "start from the robot's current state". Without it the
        # request carries an empty start state, and move_group logs
        #   [conversions]: Found empty JointState message
        # on every plan while quietly falling back to the current state.
        goal.request.start_state.is_diff = True
        goal.planning_options.plan_only = plan_only
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.print_status()
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
        name = _ERROR_CODES.get(code, f"unrecognised({code})")
        planned = len(result.planned_trajectory.joint_trajectory.points)
        print(f"\nerror_code {code} ({name})")
        # ROS 2 added message/source to MoveItErrorCodes; when populated they
        # say more than the code does.
        for field in ("message", "source"):
            text = getattr(result.error_code, field, "")
            if text:
                print(f"  {field}: {text}")
        print(f"planned trajectory: {planned} points, "
              f"planning time {result.planning_time:.3f}s")
        if code == 1 and not plan_only:
            reached = self.wait_for_joint_states(list(targets), timeout=3.0)
            print("final positions:")
            for jname, position in targets.items():
                err = math.degrees(reached[jname] - position)
                print(f"  {jname:14s} {math.degrees(reached[jname]):8.2f}°  "
                      f"error {err:+6.2f}°")
        elif code in _HINTS:
            print()
            # Say what the driver reports before offering guesses about it.
            status = self.print_status("at failure, ")
            if status is not None and code == -4:
                if status.fault:
                    print("\nThe arm IS in fault, which is cause 1 below:")
                    print("  ros2 service call "
                          f"/{group}/clear_fault std_srvs/srv/Trigger")
                elif not status.operational:
                    print("\nThe arm is NOT operational, so the driver "
                          "rejected the trajectory.")
                else:
                    print("\nThe arm is operational and unfaulted, so the "
                          "driver either rejected the goal for another reason "
                          "or could not converge within its goal tolerance — "
                          "its own log says which. Under joint impedance a "
                          "residual larger than goal_joint_tolerance_impedance "
                          "is usually gravity sag from an undeclared tool.")
            print()
            print(_HINTS[code])
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
    ap.add_argument("--random", action="store_true",
                    help="random target on all 7 joints, within --random-range "
                         "of the current pose and inside the URDF limits")
    ap.add_argument("--random-range", type=float, default=25.0,
                    help="max per-joint deviation for --random, degrees "
                         "(default 25)")
    ap.add_argument("--limit-margin", type=float, default=5.0,
                    help="keep --random targets this far inside each joint "
                         "limit, degrees (default 5)")
    ap.add_argument("--seed", type=int,
                    help="seed for --random, so --plan-only and the execute "
                         "run aim at the same pose. Printed either way.")
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

    chosen = [n for n, v in (("--named", args.named),
                             ("--joint", args.joint is not None),
                             ("--random", args.random)) if v]
    if not chosen:
        raise SystemExit(
            "give one of --named <state>, --joint <n>, --random (or --list)")
    if len(chosen) > 1:
        raise SystemExit(f"{' and '.join(chosen)} are mutually exclusive")
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
    node = GoalSender(args.group)
    try:
        if args.named:
            targets = states[args.named]
        elif args.random:
            limits, urdf = load_joint_limits(joint_names)
            seed = args.seed if args.seed is not None \
                else random.randrange(1 << 30)
            print(f"limits: {urdf}")
            print(f"seed:   {seed}   (--seed {seed} repeats this pose)")
            current = node.wait_for_joint_states(joint_names)
            targets = random_targets(current, limits, args.random_range,
                                     args.limit_margin, random.Random(seed))
        else:
            if not 1 <= args.joint <= 7:
                raise SystemExit("--joint must be 1..7")
            current = node.wait_for_joint_states(joint_names)
            name = joint_names[args.joint - 1]
            targets = {name: current[name] + math.radians(args.degrees)}
        # Populate current positions for the printout even on a named goal.
        # Same timeout as the --joint path: a freshly started node needs a
        # moment to discover /joint_states publishers, and 2s was short enough
        # that this silently printed "?" for every joint.
        try:
            node.wait_for_joint_states(joint_names, timeout=5.0)
        except TimeoutError as exc:
            print(f"note: {exc}\n      current positions unavailable; the goal "
                  f"is still absolute so this is cosmetic")
        return node.send(args.group, targets, args.plan_only, args)
    except (RuntimeError, TimeoutError) as exc:
        print(f"\nFAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
