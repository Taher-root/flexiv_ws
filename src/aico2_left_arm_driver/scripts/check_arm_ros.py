#!/usr/bin/env python3
"""Check the ROS side of one arm, and optionally command it through the driver.

Unlike the other rdk_*.py scripts, this one talks ROS, not RDK directly. Every
message type it uses is a standard ROS package, so sourcing ROS is enough --
it does not need the workspace built or sourced (though sourcing the workspace
works too, and is what you are likely to have):

    source /opt/ros/jazzy/setup.bash
    python3 check_arm_ros.py                       # checks only
    python3 check_arm_ros.py --send-goal --yes-move

It answers "does the ROS arm stack work" in the order things actually fail:

  1. Is the driver node running, in the namespace we expect?
  2. Is its lifecycle state `active`? An `unconfigured` or `inactive` driver
     publishes nothing and rejects every goal.
  3. Is /joint_states flowing, and does it carry this arm's joints? Reports the
     rate and names every publisher contributing to the topic -- with the
     merger still running there is more than one, and last writer wins.
  4. Is the `follow_joint_trajectory` action server up?
  5. With --send-goal: build a two-point trajectory from the live joint
     positions and send it, reporting acceptance, result code and the actual
     joint movement.

This is the same action MoveIt would drive, so a pass here is the prerequisite
for MoveIt rather than a substitute: `move_group` ultimately just sends
FollowJointTrajectory goals to this server. See docs/moveit_status.md for what
is and is not present on the planning side.
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import re
import xml.etree.ElementTree as ET

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

_PASS, _FAIL, _WARN = "PASS", "FAIL", "warn"


class ArmChecker(Node):
    def __init__(self, namespace, node_name, joint_names):
        super().__init__("check_arm_ros")
        self._ns = namespace.strip("/")
        self._node_name = node_name
        self._joint_names = joint_names
        self._samples = []
        self._sub = self.create_subscription(
            JointState, "/joint_states", self._on_js, qos_profile_sensor_data)
        self._fault = None
        self.create_subscription(
            Bool, f"/{self._ns}/fault" if self._ns else "/fault",
            self._on_fault, 10)
        # robot_description is latched (transient local), so a late subscriber
        # still gets it. robot_state_publisher and move_group both publish it.
        self._urdf = None
        self.create_subscription(
            String, "/robot_description", self._on_urdf,
            QoSProfile(depth=1,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.results = []

    def _on_fault(self, msg):
        self._fault = bool(msg.data)

    def _on_urdf(self, msg):
        self._urdf = msg.data

    def _on_js(self, msg: JointState):
        self._samples.append((time.monotonic(), list(msg.name),
                              list(msg.position)))

    def record(self, verdict, label, detail=""):
        self.results.append((verdict, label, detail))
        mark = {"PASS": "  ok  ", "FAIL": " FAIL ", "warn": " warn "}[verdict]
        print(f"[{mark}] {label}")
        if detail:
            for line in detail.splitlines():
                print(f"         {line}")
        return verdict == _PASS

    # ── checks ───────────────────────────────────────────────────────────────
    def check_node(self):
        want_ns = "/" + self._ns if self._ns else "/"
        found = self.get_node_names_and_namespaces()
        matches = [(n, ns) for n, ns in found if n == self._node_name]
        if not matches:
            names = ", ".join(sorted(f"{ns.rstrip('/')}/{n}"
                                     for n, ns in found)) or "(none)"
            return self.record(
                _FAIL, f"driver node {self._node_name!r} is running",
                f"not found. Nodes visible: {names}\n"
                f"Launch it: ros2 launch flexiv_amr_bringup arms.launch.py")
        ns = matches[0][1]
        if ns.rstrip("/") != want_ns.rstrip("/"):
            # Not a warning: every service and action name below is built from
            # the namespace, so all of them would be wrong too.
            return self.record(
                _FAIL, f"driver node {self._node_name!r} is running",
                f"found in namespace {ns!r}, expected {want_ns!r} — "
                f"re-run with --namespace {ns.strip('/')}")
        return self.record(_PASS, f"driver node {self._node_name!r} is running",
                           f"namespace {ns}")

    def check_lifecycle(self, timeout=5.0):
        service = f"/{self._ns}/{self._node_name}/get_state" if self._ns \
            else f"/{self._node_name}/get_state"
        client = self.create_client(GetState, service)
        if not client.wait_for_service(timeout_sec=timeout):
            return self.record(
                _FAIL, "lifecycle state is active",
                f"{service} did not appear in {timeout:.0f}s — is this a "
                f"lifecycle node?")
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            return self.record(_FAIL, "lifecycle state is active",
                               f"{service} did not answer")
        label = future.result().current_state.label
        if label != "active":
            return self.record(
                _FAIL, "lifecycle state is active",
                f"state is {label!r}. An inactive driver publishes nothing and "
                f"rejects every goal.\n"
                f"ros2 lifecycle set /{self._ns}/{self._node_name} configure\n"
                f"ros2 lifecycle set /{self._ns}/{self._node_name} activate")
        return self.record(_PASS, "lifecycle state is active")

    def check_not_faulted(self, seconds=2.0):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and self._fault is None:
            rclpy.spin_once(self, timeout_sec=0.05)
        node = f"/{self._ns}/{self._node_name}" if self._ns else f"/{self._node_name}"
        if self._fault is None:
            return self.record(
                _WARN, "arm is not in fault",
                f"no message on /{self._ns}/fault within {seconds:.0f}s")
        if self._fault:
            return self.record(
                _FAIL, "arm is not in fault",
                "the arm has a fault, so the driver rejects every trajectory "
                "goal. Clear it:\n"
                f"ros2 service call {node}/clear_fault std_srvs/srv/Trigger\n"
                "A minor fault usually means a command went out of range or hit "
                "a limit; check Flexiv Elements' event log for which.")
        return self.record(_PASS, "arm is not in fault")

    def joint_limits(self, seconds=3.0):
        """Joint limits from the URDF on /robot_description, or None."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and self._urdf is None:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self._urdf is None:
            return None
        limits = {}
        for m in re.finditer(
            r'<joint name="([^"]+)"[^>]*>(.*?)</joint>', self._urdf, re.S
        ):
            lim = re.search(r'<limit[^/]*lower="([-\d.eE]+)"\s+'
                            r'upper="([-\d.eE]+)"', m.group(2))
            if lim:
                limits[m.group(1)] = (float(lim.group(1)), float(lim.group(2)))
        return limits or None

    def check_joint_states(self, seconds=3.0):
        self._samples.clear()
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not self._samples:
            pubs = self.count_publishers("/joint_states")
            return self.record(
                _FAIL, "/joint_states is flowing",
                f"no messages in {seconds:.0f}s ({pubs} publisher(s) "
                f"advertised). Check ROS_DOMAIN_ID and that the driver is "
                f"active.")
        span = self._samples[-1][0] - self._samples[0][0]
        rate = (len(self._samples) - 1) / span if span > 0 else float("nan")
        seen = set()
        for _, names, _ in self._samples:
            seen.update(names)
        missing = [j for j in self._joint_names if j not in seen]
        detail = (f"{len(self._samples)} msgs in {span:.1f}s "
                  f"({rate:.1f} Hz aggregate across "
                  f"{self.count_publishers('/joint_states')} publisher(s))\n"
                  f"joints seen: {len(seen)}")
        if missing:
            return self.record(
                _FAIL, "/joint_states carries this arm's joints",
                detail + f"\nMISSING: {', '.join(missing)}")
        return self.record(_PASS, "/joint_states carries this arm's joints",
                           detail)

    def latest_positions(self):
        """Most recent position for each of this arm's joints."""
        out = {}
        for _, names, positions in self._samples:
            for n, p in zip(names, positions):
                if n in self._joint_names:
                    out[n] = p
        return [out[n] for n in self._joint_names] if len(out) == len(
            self._joint_names) else None

    def check_action_server(self, timeout=5.0):
        action = f"/{self._ns}/follow_joint_trajectory" if self._ns \
            else "/follow_joint_trajectory"
        self._action = ActionClient(self, FollowJointTrajectory, action)
        if not self._action.wait_for_server(timeout_sec=timeout):
            return self.record(
                _FAIL, f"action server {action} is up",
                f"no server in {timeout:.0f}s. This is the action MoveIt would "
                f"drive, so nothing downstream can work without it.")
        return self.record(_PASS, f"action server {action} is up")

    # ── the goal ─────────────────────────────────────────────────────────────
    def send_goal(self, joint, degrees, duration, timeout):
        start = self.latest_positions()
        if start is None:
            return self.record(_FAIL, "trajectory goal accepted",
                               "no live joint positions to build a goal from")
        index = joint - 1
        name = self._joint_names[index]
        target = list(start)
        target[index] = start[index] + math.radians(degrees)

        limits = self.joint_limits()
        if limits is None:
            self.record(
                _WARN, "target is within joint limits",
                "no URDF on /robot_description, so limits were not checked — "
                "start robot_state_publisher "
                "(ros2 launch flexiv_amr_description display.launch.py)")
        elif name in limits:
            lower, upper = limits[name]
            if not lower <= target[index] <= upper:
                return self.record(
                    _FAIL, "target is within joint limits",
                    f"{name} would be commanded to "
                    f"{math.degrees(target[index]):.2f}°, outside its limits "
                    f"[{math.degrees(lower):.2f}, {math.degrees(upper):.2f}]°. "
                    f"Commanding past a limit faults the arm.\n"
                    f"It is at {math.degrees(start[index]):.2f}° — try "
                    f"--degrees {-degrees:.0f}.")
            self.record(
                _PASS, "target is within joint limits",
                f"{math.degrees(target[index]):.2f}° in "
                f"[{math.degrees(lower):.2f}, {math.degrees(upper):.2f}]°")

        traj = JointTrajectory()
        traj.joint_names = list(self._joint_names)
        # Point 0 at the current pose, so the driver interpolates from where the
        # arm actually is rather than jumping to the first waypoint.
        p0 = JointTrajectoryPoint()
        p0.positions = list(start)
        p0.time_from_start = Duration(sec=0, nanosec=0)
        p1 = JointTrajectoryPoint()
        p1.positions = target
        p1.time_from_start = Duration(sec=int(duration),
                                      nanosec=int((duration % 1) * 1e9))
        traj.points = [p0, p1]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        print(f"\nsending {self._joint_names[index]} "
              f"{math.degrees(start[index]):.2f}° -> "
              f"{math.degrees(target[index]):.2f}° over {duration:.1f}s")
        send_future = self._action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return self.record(
                _FAIL, "trajectory goal accepted",
                "rejected. The driver rejects a goal when teleop is active, "
                "the arm is not operational, the hardware is not ready, or a "
                "joint name is missing — its log says which.")
        self.record(_PASS, "trajectory goal accepted")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future,
                                         timeout_sec=duration + timeout)
        if result_future.result() is None:
            return self.record(_FAIL, "trajectory completed",
                               f"no result within {duration + timeout:.0f}s")
        wrapped = result_future.result()
        res = wrapped.result
        code = res.error_code
        status = wrapped.status
        # SUCCESSFUL is 0, which is also the field's default, so a server that
        # aborts without setting error_code looks successful. Trust the goal
        # status, and treat any error_string as a failure too.
        ok = (code == FollowJointTrajectory.Result.SUCCESSFUL
              and status == GoalStatus.STATUS_SUCCEEDED
              and not res.error_string)
        status_name = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }.get(status, f"status({status})")
        detail = f"error_code={code}, goal status={status_name}"
        if res.error_string:
            detail += f"\nerror_string={res.error_string!r}"

        # Let the last joint_states arrive, then report what actually moved.
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
        end = self.latest_positions()
        if end:
            moved = math.degrees(end[index] - start[index])
            detail += (f"\n{self._joint_names[index]} moved {moved:+.2f}° "
                       f"(commanded {degrees:+.2f}°)")
        return self.record(_PASS if ok else _FAIL, "trajectory completed",
                           detail)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--namespace", default="left_arm",
                    help="driver namespace (left_arm or right_arm)")
    ap.add_argument("--node-name", default=None,
                    help="defaults to <namespace>_driver")
    ap.add_argument("--prefix", default=None,
                    help="joint name prefix; defaults to Left/Right from the "
                         "namespace")
    ap.add_argument("--seconds", type=float, default=3.0,
                    help="/joint_states sampling window")
    ap.add_argument("--send-goal", action="store_true",
                    help="also send a trajectory goal (MOVES THE ARM)")
    ap.add_argument("--yes-move", action="store_true",
                    help="required with --send-goal")
    ap.add_argument("--joint", type=int, default=4,
                    help="arm joint to move, 1-7 (default 4, the elbow — "
                         "visible, unlike joint 7)")
    ap.add_argument("--degrees", type=float, default=-15.0,
                    help="relative move in degrees; negative by default because "
                         "Left_joint4's upper limit (159°) is close to the poses "
                         "this arm tends to sit in")
    ap.add_argument("--duration", type=float, default=4.0,
                    help="trajectory duration in seconds")
    ap.add_argument("--timeout", type=float, default=5.0)
    args = ap.parse_args()

    if args.send_goal and not args.yes_move:
        print("--send-goal moves the arm. Add --yes-move once the E-stop is in")
        print("your hand and the arm has clearance.")
        return 0
    if not 1 <= args.joint <= 7:
        raise SystemExit("--joint must be 1..7")

    node_name = args.node_name or f"{args.namespace}_driver"
    prefix = args.prefix or ("Right" if "right" in args.namespace.lower()
                             else "Left")
    joint_names = [f"{prefix}_joint{i}" for i in range(1, 8)]

    print(f"namespace   /{args.namespace}")
    print(f"node        {node_name}")
    print(f"joints      {joint_names[0]}..{joint_names[-1]}")
    print()

    rclpy.init()
    checker = ArmChecker(args.namespace, node_name, joint_names)
    try:
        # Give discovery a moment; a fresh node sees nothing for the first
        # fraction of a second and that reads as a spurious failure.
        for _ in range(20):
            rclpy.spin_once(checker, timeout_sec=0.05)

        ok = checker.check_node()
        ok = checker.check_lifecycle() and ok
        ok = checker.check_not_faulted() and ok
        ok = checker.check_joint_states(args.seconds) and ok
        ok = checker.check_action_server(args.timeout) and ok

        if args.send_goal:
            if not ok:
                print("\nSkipping the goal: fix the failures above first.")
            else:
                ok = checker.send_goal(args.joint, args.degrees,
                                       args.duration, args.timeout) and ok

        print()
        failed = [l for v, l, _ in checker.results if v == _FAIL]
        if failed:
            print(f"{len(failed)} check(s) FAILED: {'; '.join(failed)}")
            return 1
        warned = [l for v, l, _ in checker.results if v == _WARN]
        print("All checks passed." + (f" ({len(warned)} warning(s))"
                                      if warned else ""))
        return 0
    finally:
        checker.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
