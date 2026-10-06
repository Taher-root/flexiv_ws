#!/usr/bin/env python3
"""Send a small single-joint FollowJointTrajectory goal. The ROS-side twin of
traj_publish.

A goal that rt_bridge accepts has to cover every joint the server commands,
start at the measured position, and start and end at rest -- so it cannot
reasonably be typed as YAML on a command line. This reads /joint_states for the
current position, builds a quintic ease on one joint, and sends it.

Quintic rather than linear because the bridge requires velocities and
accelerations and the sampler reproduces them exactly: a quintic ease is zero in
both at each end, which is the shape MoveIt itself produces after Ruckig.

Nothing moves without --yes-move, and travel is capped.

    ros2 run aico2_rt_control send_goal.py --list
    ros2 run aico2_rt_control send_goal.py --joint Left_joint6 --degrees 3 --yes-move
    ros2 run aico2_rt_control send_goal.py --joint-index 6 --degrees -5 --seconds 6 --yes-move
"""

import argparse
import math
import sys

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

DEG = math.pi / 180.0
MAX_DEGREES_DEFAULT = 15.0


def quintic(u):
    """Position, d/du and d2/du2 of the 5th-order ease. Zero rates at u=0,1."""
    return (
        10 * u**3 - 15 * u**4 + 6 * u**5,
        30 * u**2 - 60 * u**3 + 30 * u**4,
        60 * u - 180 * u**2 + 120 * u**3,
    )


class Sender(Node):
    def __init__(self, args):
        super().__init__("send_goal")
        self.args = args
        self.state = None
        self.create_subscription(JointState, "joint_states", self._on_state, 10)
        self.client = ActionClient(self, FollowJointTrajectory, "follow_joint_trajectory")

    def _on_state(self, msg):
        if self.state is None:
            self.state = msg

    def wait_for_state(self, timeout=5.0):
        end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while self.state is None and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.state is not None

    def build(self, index, delta, seconds, n_points):
        """All joints held at their measured value, one swept."""
        names = list(self.state.name)
        q0 = list(self.state.position)
        traj = JointTrajectory()
        traj.joint_names = names
        for i in range(n_points):
            u = i / (n_points - 1)
            e, de, dde = quintic(u)
            p = JointTrajectoryPoint()
            p.positions = list(q0)
            p.velocities = [0.0] * len(names)
            p.accelerations = [0.0] * len(names)
            p.positions[index] = q0[index] + delta * e
            p.velocities[index] = delta * de / seconds
            p.accelerations[index] = delta * dde / (seconds**2)
            t = seconds * u
            p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
            traj.points.append(p)
        return traj


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true", help="show joints and positions, send nothing")
    ap.add_argument("--joint", help="joint name, e.g. Left_joint6")
    ap.add_argument("--joint-index", type=int, help="index instead of a name")
    ap.add_argument("--degrees", type=float, default=0.0, help="travel, signed")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--points", type=int, default=24)
    ap.add_argument("--max-degrees", type=float, default=MAX_DEGREES_DEFAULT)
    ap.add_argument("--yes-move", action="store_true", help="required; the arm WILL move")
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=sys.argv)
    node = Sender(args)

    if not node.wait_for_state():
        node.get_logger().error(
            "no /joint_states in 5 s. Is rt_bridge running, and is this the right namespace? "
            "Try: --ros-args -r __ns:=/left_arm")
        return 1

    names = list(node.state.name)
    print("\nindex  joint                 position (deg)")
    for i, (nm, q) in enumerate(zip(names, node.state.position)):
        print(f"  [{i}]  {nm:<20s} {q / DEG:8.2f}")
    print()

    if args.list:
        return 0

    if args.joint is not None:
        if args.joint not in names:
            print(f"'{args.joint}' is not in /joint_states", file=sys.stderr)
            return 1
        index = names.index(args.joint)
    elif args.joint_index is not None:
        index = args.joint_index
        if not 0 <= index < len(names):
            print(f"--joint-index must be 0..{len(names) - 1}", file=sys.stderr)
            return 1
    else:
        print("give --joint or --joint-index (or --list)", file=sys.stderr)
        return 1

    if abs(args.degrees) > args.max_degrees:
        print(f"refusing {args.degrees:.1f} deg: over the {args.max_degrees:.0f} deg cap. "
              "Raise --max-degrees deliberately if that is really wanted.", file=sys.stderr)
        return 1
    if args.seconds <= 0.2 or not 4 <= args.points <= 4096:
        print("--seconds must exceed 0.2 and --points be 4..4096", file=sys.stderr)
        return 1

    delta = args.degrees * DEG
    q_now = node.state.position[index]
    print(f"{names[index]}: {q_now / DEG:.2f} -> {(q_now + delta) / DEG:.2f} deg "
          f"over {args.seconds:.1f} s, {args.points} points, all {len(names)} joints included.")

    if not args.yes_move:
        print("\nNot sent: --yes-move was not given. The arm WILL move with it.")
        return 0

    if not node.client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("no follow_joint_trajectory action server")
        return 1

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = node.build(index, delta, args.seconds, args.points)

    print("sending...")
    send = node.client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send)
    handle = send.result()
    if handle is None or not handle.accepted:
        print("REJECTED by the bridge. Its log says why -- most often the start position "
              "does not match, or the waist is pinned.", file=sys.stderr)
        return 1
    print("accepted; waiting...")

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    res = result_future.result()
    if res is None:
        print("no result returned", file=sys.stderr)
        return 1

    code = res.result.error_code
    print(f"\nerror_code {code} ({'SUCCESS' if code == 0 else 'FAILURE'})")
    if res.result.error_string:
        print(f"error_string: {res.result.error_string}")

    rclpy.spin_once(node, timeout_sec=0.5)
    final = node.state.position[index] if node.state else float("nan")
    want = (q_now + delta) / DEG
    print(f"commanded {want:.3f} deg, measured {final / DEG:.3f} deg, "
          f"error {final / DEG - want:.3f} deg")
    return 0 if code == 0 else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    finally:
        if rclpy.ok():
            rclpy.shutdown()
