#!/usr/bin/env python3
"""Move one waist axis (AGV_Joint1 yaw / AGV_Joint2 pitch) and come back.

The waist is the two EXTERNAL axes of the Rizon controller, q[0:2] of the
9-element state vector -- q[2:9] is the arm. Nothing else in this workspace
commands them: aico2_waist_driver only reads them, arm_driver_node's
_expand_arm_to_rdk deliberately re-sends their measured values so the waist
holds still while the arm moves, and rdk_joint_move_demo.py refuses indices
outside 1..DoF_m. So this is the one place they are written.

Measured on Rizon4-063352, 2026-09-30:
    AGV_Joint1 (yaw)    range -87.45 .. +87.45 deg
    AGV_Joint2 (pitch)  range  +2.50 .. +87.45 deg   <- minimum is NOT zero

HOW IT WORKS
  SendJointPosition takes the full robot DoF vector, so moving the waist is
  just putting a different number in q[0] or q[1] while every arm entry keeps
  its measured value. The only extra step is LockExternalAxes(False), which
  the RDK requires in IDLE -- hence before SwitchMode, the same ordering
  arm_driver_node uses for servo and Cartesian modes.

WHY ONE AXIS AT A TIME
  The waist carries the whole torso and both arms. Moving one axis by a few
  degrees is a large displacement at the end effectors and a lot of moving
  mass. There is no --both.

SAFETY
  --yes-move is required. The default is 3 degrees. The target is validated
  against q_min/q_max with a margin before the arm is enabled, so a bad
  number is refused while the brakes are still on. Velocity and acceleration
  are capped far below the arm's limits. Ctrl-C stops the robot.

Usage:
    python3 rdk_waist_move.py <sn>                        # dry run, no motion
    python3 rdk_waist_move.py <sn> --axis pitch --degrees 3 --yes-move
    python3 rdk_waist_move.py <sn> --axis yaw --degrees -5 --yes-move
    python3 rdk_waist_move.py <sn> --axis pitch --degrees 3 --yes-move --stay
"""
import argparse
import math
import sys
import time

from rdk_common import (
    connect,
    deg,
    enable,
    fmt,
    install_sigint_handler,
    lock_external_axes,
    stop_robot,
)

AXES = {"yaw": 0, "pitch": 1}     # index into the 9-element q vector
LIMIT_MARGIN_DEG = 2.0            # stay this far inside q_min/q_max
MAX_VEL = 0.2                     # rad/s, every axis
MAX_ACC = 0.4                     # rad/s^2
SETTLE_TOL_DEG = 0.3
SETTLE_TIMEOUT = 20.0


def hold_vector(robot, dof):
    """Current q/dq, which is what every non-moving axis gets commanded to."""
    st = robot.states()
    return ([float(x) for x in st.q[:dof]], [0.0] * dof)


def go_to(robot, dof, index, target_rad, label):
    """Command one axis to target while every other axis holds, then wait."""
    deadline = time.monotonic() + SETTLE_TIMEOUT
    tol = math.radians(SETTLE_TOL_DEG)
    last_print = 0.0
    while time.monotonic() < deadline:
        q_hold, dq = hold_vector(robot, dof)
        err = target_rad - q_hold[index]
        if abs(err) < tol:
            print(f"  {label}: reached {deg(q_hold[index]):.2f}°")
            return True
        q_cmd = list(q_hold)
        q_cmd[index] = target_rad
        robot.SendJointPosition(q_cmd, dq, [MAX_VEL] * dof, [MAX_ACC] * dof)
        now = time.monotonic()
        if now - last_print > 0.5:
            print(f"  {label}: at {deg(q_hold[index]):7.2f}°  "
                  f"target {deg(target_rad):7.2f}°  "
                  f"err {deg(err):+6.2f}°")
            last_print = now
        time.sleep(0.02)
    print(f"  {label}: TIMED OUT after {SETTLE_TIMEOUT:.0f}s", file=sys.stderr)
    return False


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("robot_sn")
    ap.add_argument("--axis", choices=sorted(AXES), default="pitch",
                    help="yaw = AGV_Joint1, pitch = AGV_Joint2 (default pitch)")
    ap.add_argument("--degrees", type=float, default=3.0,
                    help="relative move from where the axis is now (default 3)")
    ap.add_argument("--stay", action="store_true",
                    help="do not return to the starting angle")
    ap.add_argument("--yes-move", action="store_true",
                    help="required: this moves the torso and both arms")
    ap.add_argument("--enable-timeout", type=float, default=30.0)
    args = ap.parse_args()

    install_sigint_handler()
    rdk, robot = connect(args.robot_sn)

    info = robot.info()
    dof = int(info.DoF)
    ext_dof = int(getattr(info, "DoF_e", 0) or 0)
    if ext_dof < 2:
        raise SystemExit(f"robot reports DoF_e={ext_dof}; no waist to move")

    index = AXES[args.axis]
    st = robot.states()
    start = float(st.q[index])
    target = start + math.radians(args.degrees)
    lo = float(info.q_min[index]) + math.radians(LIMIT_MARGIN_DEG)
    hi = float(info.q_max[index]) - math.radians(LIMIT_MARGIN_DEG)

    print(f"\nDoF {dof}  DoF_m {info.DoF_m}  DoF_e {ext_dof}")
    print(f"q now {fmt([deg(x) for x in st.q[:dof]])}")
    print(f"\n{args.axis} (q[{index}])")
    print(f"  now      {deg(start):8.2f}°")
    print(f"  target   {deg(target):8.2f}°  ({args.degrees:+.2f}°)")
    print(f"  usable   {deg(lo):8.2f}° .. {deg(hi):8.2f}°  "
          f"(limits {deg(info.q_min[index]):.2f} .. "
          f"{deg(info.q_max[index]):.2f}, {LIMIT_MARGIN_DEG}° margin)")

    # Validated before Enable, so a bad target is refused with the brakes on.
    if not lo <= target <= hi:
        raise SystemExit(
            f"\ntarget {deg(target):.2f}° is outside the usable range "
            f"[{deg(lo):.2f}, {deg(hi):.2f}] -- refusing")

    if not args.yes_move:
        print("\nDry run. This would enable the robot and rotate the torso, "
              "which swings both arms.")
        print("Re-run with --yes-move once the E-stop is in your hand and the "
              "sweep is clear:\n")
        print(f"  python3 rdk_waist_move.py {args.robot_sn} "
              f"--axis {args.axis} --degrees {args.degrees} --yes-move\n")
        return 0

    enable(robot, args.enable_timeout)

    # LockExternalAxes needs IDLE, so it goes before SwitchMode -- same
    # ordering arm_driver_node._enter_servo_rdk_mode uses.
    robot.Stop()
    time.sleep(0.05)
    lock_external_axes(robot, False)
    robot.SwitchMode(rdk.Mode.NRT_JOINT_POSITION)
    print(f"mode {robot.mode()}")

    try:
        print(f"\n-> {args.degrees:+.2f}°")
        ok = go_to(robot, dof, index, target, args.axis)
        if ok and not args.stay:
            time.sleep(1.0)
            print(f"\n-> back to {deg(start):.2f}°")
            ok = go_to(robot, dof, index, start, args.axis)
    finally:
        stop_robot("waist move finished")

    q_end = float(robot.states().q[index])
    print(f"\nfinal {args.axis} {deg(q_end):.2f}°  "
          f"(started {deg(start):.2f}°)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
