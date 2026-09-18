#!/usr/bin/env python3
"""Move one arm joint in NRT_JOINT_POSITION, then in NRT_JOINT_IMPEDANCE.

Same motion both times. The point is what happens when you push the arm while
it holds the target:

  position    NRT_JOINT_POSITION. A stiff position controller. Push a link and
              it barely moves and pushes back hard. This is what the driver
              uses today for MoveIt trajectories and VR servo.
  impedance   NRT_JOINT_IMPEDANCE with SetJointImpedance(K_q). Identical
              SendJointPosition calls -- the header lists both modes as
              applicable -- but now each joint is a spring of adjustable
              stiffness. Push a link and it yields by (torque / K_q) radians
              and springs back when you let go.

That is the whole difference, and it is why MoveIt trajectories can be made
compliant without touching the trajectory code: only the mode and K_q change.

WHAT MOVES
  One joint, --joint (1-7, arm joints; default 7, the wrist, which sweeps the
  least), by --degrees (default 10) from wherever it is now, then back. The
  waist external axes are locked. Velocity and acceleration are capped well
  below the arm's limits.

  q is length 9 on this robot: q[0:2] is the waist, q[2:9] the arm. Every
  SendJointPosition vector has to be length 9 (robot DoF), so the waist entries
  are filled with their current values and locked.

STIFFNESS
  K_q_nom on this arm is [inf, inf, 6000, 6000, 4200, 4200, 1500, 1500, 1500]
  Nm/rad -- the two infinities are the waist, which is positionally rigid and
  not ours to soften. --stiffness-ratio scales only the arm entries.

  Ratio 0.0 makes the arm free-floating (the header: "Setting motion stiffness
  of a joint axis to 0 will make this axis free-floating"). Be careful with
  that on this robot: the gripper is not declared to the controller, so gravity
  compensation is ~24 N short and a free-floating arm will sag under its own
  tool. The default 0.3 is soft enough to feel by hand and stiff enough to hold
  its own weight.

SAFETY
  E-stop in hand. Robot in Auto (Remote) -- Manual mode and even plain Auto
  both refuse RDK control, and this script says so in a second rather than
  waiting out the enable timeout. Nothing runs without --yes-move.

Usage:
    python3 rdk_joint_move_demo.py <sn> position --yes-move
    python3 rdk_joint_move_demo.py <sn> impedance --yes-move
    python3 rdk_joint_move_demo.py <sn> impedance --yes-move --stiffness-ratio 0.1
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import rdk_common as rc

_SCRIPT = "rdk_joint_move_demo.py"


def _wait_until_reached(robot, target, index, tol_deg, timeout_sec, label):
    """Poll q until the moved joint is within tol_deg of its target."""
    tol = math.radians(tol_deg)
    t0 = time.monotonic()
    last_report = -1.0
    while True:
        q = [float(x) for x in robot.states().q]
        err = abs(q[index] - target[index])
        elapsed = time.monotonic() - t0
        if elapsed - last_report >= 0.5:
            last_report = elapsed
            print(f"t={elapsed:5.1f}s  {label} at "
                  f"{math.degrees(q[index]):8.2f}°  "
                  f"error {math.degrees(err):6.2f}°", flush=True)
        if err <= tol:
            print(f"reached target within {tol_deg}° after {elapsed:.1f}s")
            return q
        if robot.fault():
            raise RuntimeError("fault during motion — check the event log")
        if elapsed >= timeout_sec:
            raise TimeoutError(
                f"{label} still {math.degrees(err):.2f}° from target "
                f"after {timeout_sec:.0f}s"
            )
        time.sleep(0.05)


def _send(robot, target, max_vel, max_acc):
    """One SendJointPosition. Vectors are robot DoF (9), not arm DoF (7)."""
    robot.SendJointPosition(target, [0.0] * len(target), max_vel, max_acc)


def _stream_hold(robot, target, index, seconds, rate, banner):
    """Keep commanding the target and report how far the joint is pushed off.

    SendJointPosition re-plans on every call, so re-sending the same target is
    a hold, not a new motion.
    """
    print()
    print("=" * 72)
    print(banner)
    print("=" * 72)
    period = 1.0 / rate
    t0 = time.monotonic()
    last_report = -1.0
    peak = 0.0
    while time.monotonic() - t0 < seconds:
        q = [float(x) for x in robot.states().q]
        tau_ext = [abs(float(x)) for x in robot.states().tau_ext]
        off = math.degrees(abs(q[index] - target[index]))
        peak = max(peak, off)
        elapsed = time.monotonic() - t0
        if elapsed - last_report >= 0.5:
            last_report = elapsed
            print(f"t={elapsed:5.1f}s  pushed off target by {off:6.2f}°  "
                  f"|tau_ext|max {max(tau_ext):6.2f} Nm", flush=True)
        if robot.fault():
            raise RuntimeError("fault while holding — check the event log")
        time.sleep(period)
    print(f"\npeak deflection {peak:.2f}°")
    return peak


def _plan(args, robot):
    """Work out and validate the target before anything is enabled.

    info() and states() both work on a disabled arm, so every argument check
    and the joint-limit check happen here -- releasing the brakes only to
    reject a typo would be rude.
    """
    info = robot.info()
    dof = int(info.DoF)
    dof_m = int(getattr(info, "DoF_m", 7) or 7)
    arm_base = dof - dof_m           # 2 on this robot: q[0:2] is the waist
    if not 1 <= args.joint <= dof_m:
        raise SystemExit(f"--joint must be 1..{dof_m} (arm joints)")
    index = arm_base + (args.joint - 1)
    label = f"arm joint {args.joint} (q[{index}])"

    start = [float(x) for x in robot.states().q]
    target = list(start)
    target[index] = start[index] + math.radians(args.degrees)

    q_min = [float(x) for x in info.q_min]
    q_max = [float(x) for x in info.q_max]
    if not q_min[index] <= target[index] <= q_max[index]:
        raise SystemExit(
            f"joint {args.joint} target "
            f"{math.degrees(target[index]):.1f}° is outside its limits "
            f"[{math.degrees(q_min[index]):.1f}, "
            f"{math.degrees(q_max[index]):.1f}]° — "
            f"try --degrees {-args.degrees}"
        )

    print(f"\n{label} of {dof}")
    print(f"  start  {math.degrees(start[index]):8.2f}°")
    print(f"  target {math.degrees(target[index]):8.2f}°  "
          f"({args.degrees:+.1f}°)")
    print(f"  limits [{math.degrees(q_min[index]):.1f}, "
          f"{math.degrees(q_max[index]):.1f}]°")

    max_vel = [math.radians(args.vel_deg)] * dof
    max_acc = [math.radians(args.acc_deg)] * dof
    return start, target, index, max_vel, max_acc, label


def _prepare(args, rdk, robot, mode):
    """Plan first, then enable, lock the waist and switch mode."""
    plan = _plan(args, robot)
    rc.enable(robot, args.enable_timeout)
    # LockExternalAxes requires IDLE, so it has to happen before SwitchMode.
    rc.lock_external_axes(robot, True)
    robot.SwitchMode(mode)
    print(f"mode: {robot.mode()}")
    return plan


def stage_position(args, rdk, robot):
    start, target, index, max_vel, max_acc, label = _prepare(
        args, rdk, robot, rdk.Mode.NRT_JOINT_POSITION)

    print(f"\nmoving {args.degrees:+.1f}° at {args.vel_deg:.0f}°/s")
    _send(robot, target, max_vel, max_acc)
    _wait_until_reached(robot, target, index, args.tol_deg, args.move_timeout,
                        label)

    _stream_hold(robot, target, index, args.hold, args.rate,
                 f"PUSH THE ARM — {args.hold:.0f}s of stiff position hold")

    print(f"\nreturning to start")
    _send(robot, start, max_vel, max_acc)
    _wait_until_reached(robot, start, index, args.tol_deg, args.move_timeout,
                        label)
    print("\nThat deflection is what a position controller gives you: near "
          "zero,\nand the arm resisted you the whole time.")


def stage_impedance(args, rdk, robot):
    start, target, index, max_vel, max_acc, label = _prepare(
        args, rdk, robot, rdk.Mode.NRT_JOINT_IMPEDANCE)

    info = robot.info()
    dof = int(info.DoF)
    dof_m = int(getattr(info, "DoF_m", 7) or 7)
    arm_base = dof - dof_m
    nominal = [float(x) for x in info.K_q_nom]
    # Only the arm entries are ours: the waist nominals are inf and the valid
    # range is [0, K_q_nom], so they stay at nominal.
    K_q = list(nominal)
    for i in range(arm_base, dof):
        K_q[i] = nominal[i] * args.stiffness_ratio
    print(f"\nK_q_nom {rc.fmt(nominal, 0)}")
    print(f"K_q set {rc.fmt(K_q, 0)}  (arm scaled by "
          f"{args.stiffness_ratio})")
    if args.stiffness_ratio == 0.0:
        print("  arm joints are FREE-FLOATING. With the tool undeclared, "
              "expect it to sag.")
    robot.SetJointImpedance(K_q)

    print(f"\nmoving {args.degrees:+.1f}° at {args.vel_deg:.0f}°/s "
          f"(impedance tracking)")
    _send(robot, target, max_vel, max_acc)
    _wait_until_reached(robot, target, index, args.tol_deg, args.move_timeout,
                        label)

    k_joint = K_q[index]
    give = (math.degrees(args.probe_torque / k_joint) if k_joint else
            float("inf"))
    print(f"\njoint {args.joint} stiffness is {k_joint:.0f} Nm/rad, so "
          f"{args.probe_torque:.0f} Nm of your push should give ~{give:.2f}°")
    peak = _stream_hold(
        robot, target, index, args.hold, args.rate,
        f"PUSH THE ARM — {args.hold:.0f}s of compliant hold at "
        f"{args.stiffness_ratio}x stiffness")

    print(f"\nreturning to start and restoring nominal stiffness")
    _send(robot, start, max_vel, max_acc)
    _wait_until_reached(robot, start, index, args.tol_deg, args.move_timeout,
                        label)
    robot.SetJointImpedance(nominal)
    print(f"\nPeak deflection {peak:.2f}°, against near zero in position mode.")
    print("Same SendJointPosition calls, same trajectory code — only the mode")
    print("and K_q changed. That is the hook for compliant MoveIt motion.")


_STAGES = {"position": stage_position, "impedance": stage_impedance}


def build_parser():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("robot_sn")
    p.add_argument("stage", choices=sorted(_STAGES))
    p.add_argument("--yes-move", action="store_true",
                   help="required: both stages enable and move the arm")
    p.add_argument("--joint", type=int, default=7,
                   help="arm joint to move, 1-7 (default 7, the wrist)")
    p.add_argument("--degrees", type=float, default=10.0,
                   help="relative move in degrees (can be negative)")
    p.add_argument("--vel-deg", type=float, default=10.0,
                   help="max joint velocity, deg/s")
    p.add_argument("--acc-deg", type=float, default=30.0,
                   help="max joint acceleration, deg/s^2")
    p.add_argument("--hold", type=float, default=15.0,
                   help="seconds to hold the target so you can push it")
    p.add_argument("--rate", type=float, default=20.0,
                   help="SendJointPosition rate during the hold, Hz")
    p.add_argument("--tol-deg", type=float, default=0.5,
                   help="convergence tolerance")
    p.add_argument("--move-timeout", type=float, default=30.0)
    p.add_argument("--enable-timeout", type=float, default=30.0)
    p.add_argument("--stiffness-ratio", type=float, default=0.3,
                   help="impedance stage: fraction of K_q_nom on the arm "
                        "joints; 0 is free-floating")
    p.add_argument("--probe-torque", type=float, default=10.0,
                   help="impedance stage: the push, in Nm, whose predicted "
                        "give is printed")
    return p


def main():
    args = build_parser().parse_args()
    if not 0.0 <= args.stiffness_ratio <= 1.0:
        raise SystemExit("--stiffness-ratio must be 0..1 "
                         "(valid range for K_q is [0, K_q_nom])")
    if not 1.0 <= args.rate <= 100.0:
        raise SystemExit("--rate must be 1..100 Hz")
    rc.install_sigint_handler()

    if not rc.gate_move(args, _SCRIPT, warning_lines=(
        f"It moves arm joint {args.joint} by {args.degrees:+.1f}° and holds it "
        f"for {args.hold:.0f}s so you can push the arm by hand.",
    )):
        return 0

    try:
        rdk, robot = rc.connect(args.robot_sn)
        _STAGES[args.stage](args, rdk, robot)
    except SystemExit:
        raise
    except Exception as exc:  # noqa: BLE001
        rc.stop_robot(f"{type(exc).__name__}: {exc}")
        print(f"\nFAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        rc.stop_robot("stage finished")
    print("\nDone.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
