#!/usr/bin/env python3
"""Measure how well the arm tracks a trajectory, so "smooth" becomes a number.

Every smoothness change so far has been judged by eye, which is why the wrong
cause survived three rounds. This sends one trajectory, records /joint_states
throughout, and reports metrics that distinguish the candidate causes from each
other:

    source /opt/ros/jazzy/setup.bash
    python3 measure_tracking.py --joint 4 --degrees -15 --yes-move
    python3 measure_tracking.py --joint 4 --degrees -15 --yes-move --csv run.csv

WHAT IT REPORTS, AND WHAT EACH ONE MEANS

  tracking error (from the action's own feedback: desired vs actual)
      RMS and peak lag between where the driver said to be and where the arm
      was. Large and roughly constant => the controller is trailing the
      setpoint, which is what low stiffness does. Large only at the ends =>
      acceleration limits.

  acceleration sign reversals
      The roughness metric. A single smooth move accelerates, coasts and
      decelerates, so its acceleration goes positive, through zero, negative:
      ONE reversal. (The coast is ignored -- values under 10% of peak are
      treated as zero so sensor noise is not counted, which is also why a clean
      trapezoid scores 1 rather than 2.) Every additional reversal is a
      velocity wobble you would feel as a stutter. This is the number that
      should fall when re-plan churn is reduced -- see
      trajectory_send_rate_hz.

  peak |velocity| / |acceleration|
      Compared against what the trajectory asked for. Well above it means the
      controller is overshooting each setpoint and being pulled back, i.e.
      default_max_joint_vel/acc are far above the trajectory's own profile.

  final error and settle time
      Accuracy. In impedance mode expect a steady residual equal to
      (unmodelled torque / K_q) -- gravity sag from an undeclared tool. That
      one does not improve with tuning; it improves by declaring the tool.

CONFIGURATIONS WORTH COMPARING (same move each time)

  joint_control_mode:=position                   vs impedance
  joint_stiffness_ratio 1.0                      vs 0.3
  trajectory_send_rate_hz 50                     vs 10
  default_max_joint_acc 3.0                      vs 0.5

Change one at a time with `ros2 param set` and re-run. --csv writes the raw
samples so two runs can be plotted against each other.
"""
from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrackingRun(Node):
    def __init__(self, namespace, joint_names):
        super().__init__("measure_tracking")
        self._ns = namespace.strip("/")
        self._joint_names = joint_names
        self._samples = []          # (t, {joint: position})
        self._feedback = []         # (t, desired, actual)
        self._latest = {}
        self.create_subscription(JointState, "/joint_states", self._on_js,
                                 qos_profile_sensor_data)
        self._client = ActionClient(
            self, FollowJointTrajectory,
            f"/{self._ns}/follow_joint_trajectory" if self._ns
            else "/follow_joint_trajectory")
        self._recording = False
        self._t0 = 0.0

    def _on_js(self, msg):
        for name, position in zip(msg.name, msg.position):
            self._latest[name] = position
        if self._recording and all(n in self._latest for n in self._joint_names):
            self._samples.append((
                time.monotonic() - self._t0,
                {n: self._latest[n] for n in self._joint_names},
            ))

    def _on_feedback(self, msg):
        if not self._recording:
            return
        fb = msg.feedback
        self._feedback.append((
            time.monotonic() - self._t0,
            list(fb.desired.positions),
            list(fb.actual.positions),
        ))

    def wait_for_state(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(n in self._latest for n in self._joint_names):
                return {n: self._latest[n] for n in self._joint_names}
        raise TimeoutError("no /joint_states for all arm joints")

    def run(self, index, degrees, duration, timeout):
        start = self.wait_for_state()
        name = self._joint_names[index]
        target = dict(start)
        target[name] = start[name] + math.radians(degrees)

        if not self._client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("no follow_joint_trajectory server")

        traj = JointTrajectory()
        traj.joint_names = list(self._joint_names)
        p0 = JointTrajectoryPoint()
        p0.positions = [start[n] for n in self._joint_names]
        p0.time_from_start = Duration(sec=0, nanosec=0)
        p1 = JointTrajectoryPoint()
        p1.positions = [target[n] for n in self._joint_names]
        p1.time_from_start = Duration(sec=int(duration),
                                      nanosec=int((duration % 1) * 1e9))
        traj.points = [p0, p1]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        print(f"moving {name} {math.degrees(start[name]):.2f}° -> "
              f"{math.degrees(target[name]):.2f}° over {duration:.1f}s\n")
        self._t0 = time.monotonic()
        self._recording = True
        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("goal rejected — run check_arm_ros.py")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=duration + timeout + 30.0)
        # Keep recording briefly after the goal ends to catch settling / sag.
        settle_until = time.monotonic() + 1.5
        while time.monotonic() < settle_until:
            rclpy.spin_once(self, timeout_sec=0.02)
        self._recording = False

        wrapped = result_future.result()
        return (wrapped.result if wrapped else None,
                wrapped.status if wrapped else None,
                name, target[name])


def _derivatives(times, values):
    """Finite-difference velocity and acceleration, unevenly sampled."""
    vel, acc = [], []
    for i in range(1, len(values)):
        dt = times[i] - times[i - 1]
        vel.append((values[i] - values[i - 1]) / dt if dt > 0 else 0.0)
    for i in range(1, len(vel)):
        dt = times[i + 1] - times[i]
        acc.append((vel[i] - vel[i - 1]) / dt if dt > 0 else 0.0)
    return vel, acc


def _sign_reversals(series, deadband):
    """Sign changes, ignoring values inside a deadband so noise is not counted."""
    reversals, last = 0, 0
    for value in series:
        sign = 0 if abs(value) < deadband else (1 if value > 0 else -1)
        if sign and last and sign != last:
            reversals += 1
        if sign:
            last = sign
    return reversals


def report(run, name, target, result, status, csv_path):
    samples, feedback = run._samples, run._feedback
    if len(samples) < 5:
        print(f"only {len(samples)} samples recorded — nothing to analyse")
        return 1

    times = [t for t, _ in samples]
    values = [q[name] for _, q in samples]
    vel, acc = _derivatives(times, values)
    span = times[-1] - times[0]

    print("=" * 66)
    print(f"MOTION OF {name}")
    print("=" * 66)
    print(f"  samples            {len(samples)} over {span:.2f}s "
          f"({len(samples) / span:.1f} Hz)")
    print(f"  travelled          {math.degrees(values[-1] - values[0]):+.2f}°")
    print(f"  peak |velocity|    {math.degrees(max(abs(v) for v in vel)):.1f} °/s")
    if acc:
        print(f"  peak |accel|       "
              f"{math.degrees(max(abs(a) for a in acc)):.1f} °/s²")
        # 10% of peak keeps sensor noise from counting as a reversal.
        deadband = 0.1 * max(abs(a) for a in acc)
        reversals = _sign_reversals(acc, deadband)
        print(f"  accel reversals    {reversals}   "
              f"(1 = one smooth accelerate-coast-decelerate; "
              f"each extra one is a stutter)")

    print()
    print("=" * 66)
    print("TRACKING (driver's own desired vs actual)")
    print("=" * 66)
    if not feedback:
        print("  no feedback received — the driver publishes it per send, so")
        print("  either the move was too short or feedback is not reaching us")
    else:
        idx = run._joint_names.index(name)
        errors = [abs(d[idx] - a[idx]) for _, d, a in feedback
                  if len(d) > idx and len(a) > idx]
        if errors:
            rms = math.sqrt(sum(e * e for e in errors) / len(errors))
            print(f"  feedback samples   {len(errors)} "
                  f"({len(errors) / span:.1f} Hz)")
            print(f"  RMS lag            {math.degrees(rms):.3f}°")
            print(f"  peak lag           {math.degrees(max(errors)):.3f}°")
            print(f"  median lag         "
                  f"{math.degrees(statistics.median(errors)):.3f}°")

    print()
    print("=" * 66)
    print("ACCURACY")
    print("=" * 66)
    final = values[-1]
    print(f"  final              {math.degrees(final):.3f}°")
    print(f"  target             {math.degrees(target):.3f}°")
    print(f"  error              {math.degrees(final - target):+.3f}°")
    # Drift over the post-goal window is sag, not tracking.
    tail = [q[name] for t, q in samples if t > times[-1] - 1.0]
    if len(tail) > 2:
        print(f"  drift in last 1s   "
              f"{math.degrees(max(tail) - min(tail)):.3f}°  "
              f"(a steady residual here is gravity sag, not tracking)")
    if result is not None:
        print(f"  error_code         {result.error_code}"
              + (f", {result.error_string!r}" if result.error_string else ""))
    print(f"  goal status        {status}")

    if csv_path:
        with open(csv_path, "w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(["t", "position_rad", "position_deg",
                             "velocity_deg_s", "accel_deg_s2"])
            for i, (t, q) in enumerate(samples):
                v = math.degrees(vel[i - 1]) if 0 < i <= len(vel) else ""
                a = math.degrees(acc[i - 2]) if 1 < i <= len(acc) + 1 else ""
                writer.writerow([f"{t:.4f}", f"{q[name]:.6f}",
                                 f"{math.degrees(q[name]):.4f}", v, a])
        print(f"\nwrote {csv_path}")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--namespace", default="left_arm")
    ap.add_argument("--joint", type=int, default=4)
    ap.add_argument("--degrees", type=float, default=-15.0)
    ap.add_argument("--duration", type=float, default=4.0)
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--csv", help="write the raw samples here")
    ap.add_argument("--yes-move", action="store_true", help="required: it moves")
    args = ap.parse_args()

    if not args.yes_move:
        print("This moves the arm. Add --yes-move.")
        return 0
    if not 1 <= args.joint <= 7:
        raise SystemExit("--joint must be 1..7")

    prefix = "Right" if "right" in args.namespace.lower() else "Left"
    joint_names = [f"{prefix}_joint{i}" for i in range(1, 8)]

    rclpy.init()
    run = TrackingRun(args.namespace, joint_names)
    try:
        result, status, name, target = run.run(
            args.joint - 1, args.degrees, args.duration, args.timeout)
        return report(run, name, target, result, status, args.csv)
    except (RuntimeError, TimeoutError) as exc:
        print(f"\nFAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        run.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
