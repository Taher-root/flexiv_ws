#!/usr/bin/env python3
"""Move one arm to the SRDF `ready` pose in joint space, then release it.

For recovering an arm left near a singularity or in a pose teleop cannot start
from. A singularity only breaks Cartesian control; a joint-space move goes
straight through it, so this works where the IK and Cartesian jogging do not.

Uses NRT_JOINT_POSITION: the controller plans the motion itself, slowly
(--vel-deg, default 10 deg/s), and every joint arrives together. The waist
external axes are locked and held where they are.

THE PATH IS NOT COLLISION-CHECKED. Each joint goes straight to its target, so
from a bad pose the arm can sweep through the torso or the other arm. The plan
is printed first; watch it with the E-stop in hand, and use --joints to move
only some of them (e.g. --joints 6 for the wrist alone).

PRECONDITIONS
  Nothing else connected to this arm: stop its rt_server / driver first.
  E-stop released, robot in Auto (Remote).

Usage:
    python3 rdk_goto_ready.py Rizon4R-062102                 # print the plan
    python3 rdk_goto_ready.py Rizon4R-062102 --yes-move      # move all 7
    python3 rdk_goto_ready.py Rizon4R-062102 --joints 6 --yes-move
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import rdk_common as rc

# aico2_moveit_config/config/aico2.srdf, group_state "ready" (both arms):
# joint6 = +90 deg keeps the wrist away from its singularity at 0.
READY = [0.0, -0.7145, 0.0, 1.6559, 0.0, 1.5708, 0.0]


def build_parser():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("robot_sn")
    p.add_argument("--yes-move", action="store_true",
                   help="required to enable and move the arm")
    p.add_argument("--joints", type=int, nargs="+", default=list(range(1, 8)),
                   help="arm joints to move, 1-7 (default all); the rest hold")
    p.add_argument("--vel-deg", type=float, default=10.0,
                   help="max joint velocity, deg/s")
    p.add_argument("--acc-deg", type=float, default=20.0,
                   help="max joint acceleration, deg/s^2")
    p.add_argument("--tol-deg", type=float, default=0.5)
    p.add_argument("--move-timeout", type=float, default=90.0)
    p.add_argument("--enable-timeout", type=float, default=30.0)
    return p


def plan(args, robot):
    info = robot.info()
    dof = int(info.DoF)
    dof_m = int(getattr(info, "DoF_m", 7) or 7)
    base = dof - dof_m            # 2 on this robot: q[0:2] is the waist
    if dof_m != len(READY):
        raise SystemExit(f"expected a 7-joint arm, robot reports {dof_m}")
    if any(not 1 <= j <= dof_m for j in args.joints):
        raise SystemExit("--joints must be 1..7")

    start = [float(x) for x in robot.states().q]
    target = list(start)
    for j in args.joints:
        target[base + j - 1] = READY[j - 1]

    q_min = [float(x) for x in info.q_min]
    q_max = [float(x) for x in info.q_max]
    print(f"\n{'joint':>6} {'now':>9} {'ready':>9} {'move':>8}")
    for j in range(1, dof_m + 1):
        i = base + j - 1
        moves = j in args.joints
        print(f"{j:>6} {math.degrees(start[i]):8.1f}° "
              f"{math.degrees(target[i]):8.1f}° "
              f"{math.degrees(target[i] - start[i]):+7.1f}°"
              f"{'' if moves else '   (held)'}")
        if not q_min[i] <= target[i] <= q_max[i]:
            raise SystemExit(f"joint {j} target is outside its limits")
    largest = max(abs(target[i] - start[i]) for i in range(base, dof))
    print(f"\nlargest move {math.degrees(largest):.1f}° at {args.vel_deg:.0f}°/s "
          f"(~{math.degrees(largest) / args.vel_deg:.0f}s)")
    max_vel = [math.radians(args.vel_deg)] * dof
    max_acc = [math.radians(args.acc_deg)] * dof
    return target, base, dof, max_vel, max_acc


def wait_reached(robot, target, base, dof, tol_deg, timeout_sec):
    tol = math.radians(tol_deg)
    t0 = time.monotonic()
    last = -1.0
    while True:
        q = [float(x) for x in robot.states().q]
        err = max(abs(q[i] - target[i]) for i in range(base, dof))
        elapsed = time.monotonic() - t0
        if elapsed - last >= 1.0:
            last = elapsed
            print(f"t={elapsed:5.1f}s  worst joint {math.degrees(err):6.2f}° "
                  "from ready", flush=True)
        if err <= tol:
            print(f"at ready (within {tol_deg}°) after {elapsed:.1f}s")
            return
        if robot.fault():
            raise RuntimeError("fault during motion; check the event log")
        if elapsed >= timeout_sec:
            raise TimeoutError(f"still {math.degrees(err):.1f}° away after "
                               f"{timeout_sec:.0f}s")
        time.sleep(0.05)


def main():
    args = build_parser().parse_args()
    rc.install_sigint_handler()
    try:
        rdk, robot = rc.connect(args.robot_sn)
        target, base, dof, max_vel, max_acc = plan(args, robot)
        if not args.yes_move:
            print("\nNothing moved. Re-run with --yes-move, E-stop in hand.")
            return 0
        rc.enable(robot, args.enable_timeout)
        rc.lock_external_axes(robot, True)   # needs IDLE: before SwitchMode
        robot.SwitchMode(rdk.Mode.NRT_JOINT_POSITION)
        robot.SendJointPosition(target, [0.0] * dof, max_vel, max_acc)
        wait_reached(robot, target, base, dof, args.tol_deg, args.move_timeout)
    except SystemExit:
        raise
    except Exception as exc:  # noqa: BLE001
        rc.stop_robot(f"{type(exc).__name__}: {exc}")
        print(f"\nFAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        rc.stop_robot("done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
