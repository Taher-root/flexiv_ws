#!/usr/bin/env python3
"""Make the arm compliant with the impedance API, no primitive licence needed.

The Adaptive Assembly primitives (SearchHole, CheckPiH, InsertComp, Mate,
FastenScrew) are unlicensed on this robot -- the controller logs event 303009
and refuses to load them. What that licence gated was Flexiv's packaged search
and insertion behaviour, not compliance itself. Cartesian impedance is part of
the RDK control API and works on the licence we have.

THIS SCRIPT MOVES THE ARM. Every stage is behind --yes-move.

STAGES
  soft   Soft hold. Holds the current TCP pose with reduced stiffness, so
         pushing the end effector moves it and letting go springs it back.
         This is the "push it and it complies" demo, and it is the one stage
         the 24 N gravity bias does not corrupt (see BIAS, below).
  press  Unified motion-force control: one axis force-controlled at --force
         newtons, the others holding position. What you would use to keep a
         peg pressed against a surface.
  pih    Peg-in-hole configuration: insertion axis force-controlled at a low
         force, low translational stiffness on the other two axes and low
         rotational stiffness, so the peg can slide and tilt into alignment
         instead of jamming. A hand-rolled InsertComp.

HOW COMPLIANCE IS CONFIGURED (ordering matters, from Flexiv's own
intermediate4 example)
  1. SwitchMode(NRT_CARTESIAN_MOTION_FORCE)
  2. init_pose = states().tcp_pose
  3. SetMaxContactWrench(<small>)  -- soft-contact clamp while still
     motion-controlled on every axis
  4. SetForceControlFrame(WORLD or TCP), SetForceControlAxis([...])
  5. SetMaxContactWrench([inf]*6) only AFTER force control is active on the
     axis, otherwise the contact force spikes when the clamp is lifted
  6. SendCartesianMotionForce(pose, wrench) in a loop, 1-100 Hz
  SetCartesianImpedance() can be called at any point in that loop, so
  stiffness is ramped rather than stepped.

  Motion control is always referenced to the world frame. Force control can
  reference world or TCP. Sign convention from the same example: in WORLD,
  +Fz presses down; in TCP, -Fz presses along the tool.

BIAS
  info().has_FT_sensor is False -- force is estimated from joint torques
  through the Jacobian -- and this arm reports about 24 N at rest with nothing
  touching it, almost certainly the undeclared gripper mass.

  For `soft` that does not matter much in kind, but it matters in degree.
  Impedance holds position against force, so a steady 24 N of phantom force
  displaces the TCP by 24/K metres. At the nominal 10000 N/m that is 2.4 mm
  and invisible; at 200 N/m it is 120 mm of sag the moment the stiffness
  lands. The script computes that sag from the measured resting wrench and
  refuses to ramp past --max-sag.

  For `press` and `pih` the bias is disqualifying: a commanded 5 N against a
  24 N offset is not 5 N of contact. Declare the tool in Flexiv Elements
  first. Both stages say so and require --i-know-the-bias to run anyway.

Usage:
    python3 rdk_compliance_demo.py <sn> soft --yes-move
    python3 rdk_compliance_demo.py <sn> soft --yes-move --stiffness 500
    python3 rdk_compliance_demo.py <sn> press --yes-move --i-know-the-bias
"""
from __future__ import annotations

import argparse
import math
import signal
import sys
import time

_ROBOT = None
_STOPPED = False

# Blocking operational_status() values, same set and reasons as
# rdk_primitive_pih.py: these need a person, not a wait.
_BLOCKING_STATUSES = {
    "IN_MANUAL_MODE": "switch the robot to Auto (Remote) mode",
    "IN_AUTO_MODE": "switch from regular Auto to Auto (REMOTE) mode",
    "ESTOP_NOT_RELEASED": "release the E-stop",
    "CRITICAL_FAULT": "clear the fault in Flexiv Elements",
    "IN_RECOVERY_STATE": "run recovery in Elements",
}


def _stop_robot(reason: str) -> None:
    global _STOPPED
    if _ROBOT is None or _STOPPED:
        return
    _STOPPED = True
    print(f"\n[safety] Stop() — {reason}", flush=True)
    try:
        _ROBOT.Stop()
    except Exception as exc:  # noqa: BLE001
        print(f"[safety] Stop() declined ({exc}) — nothing was moving")


def _on_sigint(_signum, _frame):
    _stop_robot("SIGINT")
    sys.exit(130)


def _fmt(vals, prec=2):
    return "[" + " ".join(f"{float(v):7.{prec}f}" for v in vals) + "]"


def _norm3(vals):
    return math.sqrt(sum(float(v) ** 2 for v in list(vals)[:3]))


def _status_name(robot):
    return str(robot.operational_status()).rsplit(".", 1)[-1]


def _resting_wrench(robot, seconds=3.0):
    n, acc = 0, [0.0] * 6
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        w = robot.states().ext_wrench_in_tcp
        acc = [a + float(b) for a, b in zip(acc, w)]
        n += 1
        time.sleep(0.002)
    return [a / max(n, 1) for a in acc]


def connect_and_enable(sn, enable_timeout):
    global _ROBOT
    import flexivrdk

    print(f"flexivrdk {getattr(flexivrdk, '__version__', '?')}")
    print(f"Connecting Robot({sn!r})")
    _ROBOT = robot = flexivrdk.Robot(sn)

    if robot.fault():
        print("fault set — ClearFault()")
        if not robot.ClearFault():
            raise RuntimeError("ClearFault() failed")

    status = _status_name(robot)
    if status in _BLOCKING_STATUSES:
        raise RuntimeError(f"robot is {status} — {_BLOCKING_STATUSES[status]}")

    print(f"status {status} — Enable()")
    robot.Enable()
    deadline = time.monotonic() + enable_timeout
    while not robot.operational():
        status = _status_name(robot)
        if status in _BLOCKING_STATUSES:
            raise RuntimeError(
                f"robot became {status} — {_BLOCKING_STATUSES[status]}")
        if time.monotonic() >= deadline:
            raise TimeoutError(f"not operational after {enable_timeout:.0f}s "
                               f"(status {status})")
        time.sleep(0.5)
    print("operational")
    return flexivrdk, robot


def check_sag(robot, target_K, max_sag, flag="--stiffness"):
    """Refuse a stiffness that turns the resting force bias into a big droop.

    Impedance holds position against force: a steady phantom force F displaces
    the TCP by F/K. The 24 N this arm reports at rest is phantom force, so the
    softer we go the further it sags.
    """
    print(f"\nMeasuring resting wrench (3s) — DO NOT TOUCH THE ARM")
    bias = _resting_wrench(robot)
    f_bias = _norm3(bias)
    k_min = min(float(k) for k in target_K[:3])
    sag = f_bias / k_min if k_min > 0 else float("inf")
    print(f"  resting |F| {f_bias:.2f} N {_fmt(bias)}")
    print(f"  softest translational stiffness {k_min:.0f} N/m")
    print(f"  predicted steady-state sag {sag * 1000:.0f} mm "
          f"({f_bias:.1f} N / {k_min:.0f} N/m)")
    if sag > max_sag:
        raise RuntimeError(
            f"predicted sag {sag * 1000:.0f} mm exceeds --max-sag "
            f"{max_sag * 1000:.0f} mm. Either raise {flag} (to at least "
            f"{f_bias / max_sag:.0f} N/m at this bias) or declare the tool in "
            f"Flexiv Elements so the bias goes away."
        )
    return bias


def enter_cartesian(rdk, robot, soft_clamp):
    """Steps 1-3 of the ordering in this file's docstring."""
    robot.SwitchMode(rdk.Mode.NRT_CARTESIAN_MOTION_FORCE)
    print(f"mode: {robot.mode()}")
    init_pose = [float(x) for x in robot.states().tcp_pose]
    print(f"init TCP pose {_fmt(init_pose, 4)}")
    robot.SetMaxContactWrench(soft_clamp)
    print(f"max contact wrench clamped to {_fmt(soft_clamp, 1)}")
    # Hold the elbow where it is: with 7 joints for a 6-DOF task the null space
    # drifts otherwise. q is length 9 here (2 waist + 7 arm), so slice the arm.
    q = [float(x) for x in robot.states().q]
    dof_m = int(getattr(robot.info(), "DoF_m", 7) or 7)
    try:
        robot.SetNullSpacePosture(q[len(q) - dof_m:])
        print("null-space posture pinned to current arm joints")
    except Exception as exc:  # noqa: BLE001
        print(f"SetNullSpacePosture skipped: {exc}")
    return init_pose


def ramp_stiffness(robot, nominal_K, target_K, ramp_sec, init_pose, rate):
    """Interpolate stiffness while streaming the hold pose.

    Stepping straight to a low stiffness drops the arm; SetCartesianImpedance
    can be called inside the streaming loop, so the drop is spread over
    ramp_sec instead.
    """
    print(f"\nramping stiffness over {ramp_sec:.1f}s")
    period = 1.0 / rate
    t0 = time.monotonic()
    while True:
        frac = min(1.0, (time.monotonic() - t0) / max(ramp_sec, 1e-6))
        K = [n + (t - n) * frac for n, t in zip(nominal_K, target_K)]
        robot.SetCartesianImpedance(K)
        robot.SendCartesianMotionForce(init_pose)
        if robot.fault():
            raise RuntimeError("fault during stiffness ramp")
        if frac >= 1.0:
            break
        time.sleep(period)
    print(f"stiffness now {_fmt(target_K, 0)}")


def stream(robot, init_pose, wrench, seconds, rate, banner):
    """Stream the hold pose and report force and TCP deviation."""
    print()
    print("=" * 72)
    print(banner)
    print("=" * 72)
    period = 1.0 / rate
    t0 = time.monotonic()
    last_print = -1.0
    peak_dev = 0.0
    peak_f = 0.0
    while time.monotonic() - t0 < seconds:
        robot.SendCartesianMotionForce(init_pose, wrench)
        if robot.fault():
            raise RuntimeError("fault while streaming — check the event log")
        st = robot.states()
        pose = [float(x) for x in st.tcp_pose]
        dev = math.sqrt(sum((a - b) ** 2 for a, b in zip(pose[:3],
                                                         init_pose[:3])))
        f = _norm3(st.ext_wrench_in_tcp)
        peak_dev = max(peak_dev, dev)
        peak_f = max(peak_f, f)
        now = time.monotonic() - t0
        if now - last_print >= 0.5:
            last_print = now
            print(f"t={now:5.1f}s  |F|={f:6.2f} N  "
                  f"TCP moved {dev * 1000:6.1f} mm from hold pose", flush=True)
        time.sleep(period)
    print(f"\npeak deviation {peak_dev * 1000:.1f} mm, peak |F| {peak_f:.2f} N")


# ── stages ───────────────────────────────────────────────────────────────────
def stage_soft(args, rdk, robot):
    nominal_K = [float(x) for x in robot.info().K_x_nom]
    target_K = [args.stiffness] * 3 + [args.rot_stiffness] * 3
    print(f"\nnominal stiffness {_fmt(nominal_K, 0)}")
    print(f"target  stiffness {_fmt(target_K, 0)}")
    check_sag(robot, target_K, args.max_sag)

    init_pose = enter_cartesian(rdk, robot, args.max_contact_wrench)
    ramp_stiffness(robot, nominal_K, target_K, args.ramp, init_pose, args.rate)
    stream(robot, init_pose, [0.0] * 6, args.seconds, args.rate,
           f"PUSH THE END EFFECTOR — {args.seconds:.0f}s at "
           f"{args.stiffness:.0f} N/m")
    print("\nIf it yielded to your hand and sprang back, that is Cartesian")
    print("impedance working. 10 N at this stiffness is "
          f"{10.0 / args.stiffness * 1000:.0f} mm of give.")


def _force_axis_flags(axis):
    idx = {"X": 0, "Y": 1, "Z": 2}[axis.lstrip("-")]
    flags = [False] * 6
    flags[idx] = True
    return flags, idx


def stage_press(args, rdk, robot):
    nominal_K = [float(x) for x in robot.info().K_x_nom]
    target_K = [args.stiffness] * 3 + [args.rot_stiffness] * 3
    check_sag(robot, target_K, args.max_sag)

    init_pose = enter_cartesian(rdk, robot, args.max_contact_wrench)
    ramp_stiffness(robot, nominal_K, target_K, args.ramp, init_pose, args.rate)

    flags, idx = _force_axis_flags(args.axis)
    frame = (rdk.CoordType.TCP if args.frame == "TCP" else rdk.CoordType.WORLD)
    robot.SetForceControlFrame(frame)
    robot.SetForceControlAxis(flags)
    print(f"force control on {args.axis} in {args.frame} frame")
    # Only now is it safe to lift the clamp: the axis is force-regulated, so
    # the contact force will not spike (intermediate4's note).
    robot.SetMaxContactWrench([float("inf")] * 6)
    init_pose = [float(x) for x in robot.states().tcp_pose]

    # WORLD +Fz presses down; TCP -Fz presses along the tool.
    magnitude = args.force if args.frame == "WORLD" else -args.force
    if args.axis.startswith("-"):
        magnitude = -magnitude
    wrench = [0.0] * 6
    wrench[idx] = magnitude
    stream(robot, init_pose, wrench, args.seconds, args.rate,
           f"PRESSING {args.force:.1f} N along {args.axis} "
           f"({args.frame} frame)")


def stage_pih(args, rdk, robot):
    """Insertion axis force-controlled, the rest soft enough to self-align."""
    nominal_K = [float(x) for x in robot.info().K_x_nom]
    # Soft laterally so the peg can slide into the hole, soft in rotation so it
    # can tilt into alignment, and the insertion axis is force-controlled so
    # its stiffness does not matter.
    target_K = [args.lateral_stiffness] * 3 + [args.rot_stiffness] * 3
    check_sag(robot, target_K, args.max_sag, "--lateral-stiffness")

    init_pose = enter_cartesian(rdk, robot, args.max_contact_wrench)
    ramp_stiffness(robot, nominal_K, target_K, args.ramp, init_pose, args.rate)

    flags, idx = _force_axis_flags(args.axis)
    robot.SetForceControlFrame(rdk.CoordType.TCP)
    robot.SetForceControlAxis(flags, [args.max_linear_vel] * 3)
    print(f"insertion axis {args.axis} force-controlled in TCP frame, "
          f"lateral stiffness {args.lateral_stiffness:.0f} N/m")
    robot.SetMaxContactWrench([float("inf")] * 6)
    init_pose = [float(x) for x in robot.states().tcp_pose]

    magnitude = -args.force if not args.axis.startswith("-") else args.force
    wrench = [0.0] * 6
    wrench[idx] = magnitude
    stream(robot, init_pose, wrench, args.seconds, args.rate,
           f"INSERTING at {args.force:.1f} N along {args.axis} (TCP frame), "
           f"compliant laterally")


_STAGES = {"soft": stage_soft, "press": stage_press, "pih": stage_pih}


def build_parser():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("robot_sn")
    p.add_argument("stage", choices=sorted(_STAGES))
    p.add_argument("--yes-move", action="store_true",
                   help="required: every stage enables and moves the arm")
    p.add_argument("--i-know-the-bias", action="store_true",
                   help="press/pih only: proceed despite the resting wrench "
                        "bias making commanded forces wrong")
    p.add_argument("--seconds", type=float, default=30.0,
                   help="how long to hold the compliant state")
    p.add_argument("--rate", type=float, default=100.0,
                   help="SendCartesianMotionForce rate, 1-100 Hz")
    p.add_argument("--ramp", type=float, default=3.0,
                   help="seconds to interpolate from nominal to target "
                        "stiffness")
    p.add_argument("--enable-timeout", type=float, default=30.0)
    p.add_argument("--stiffness", type=float, default=1000.0,
                   help="translational stiffness N/m; nominal is 10000")
    p.add_argument("--lateral-stiffness", type=float, default=300.0,
                   help="pih: stiffness on the non-insertion axes")
    p.add_argument("--rot-stiffness", type=float, default=150.0,
                   help="rotational stiffness Nm/rad; nominal is 1500")
    p.add_argument("--max-sag", type=float, default=0.05,
                   help="abort if the resting force bias would displace the "
                        "TCP by more than this (m) at the target stiffness")
    p.add_argument("--max-contact-wrench", type=float, nargs=6,
                   default=[30.0, 30.0, 30.0, 10.0, 10.0, 10.0],
                   help="soft-contact clamp while motion-controlled")
    p.add_argument("--force", type=float, default=5.0,
                   help="press/pih: commanded force in N")
    p.add_argument("--axis", choices=["X", "-X", "Y", "-Y", "Z", "-Z"],
                   default="Z", help="press/pih: force-controlled axis")
    p.add_argument("--frame", choices=["WORLD", "TCP"], default="TCP",
                   help="press: force control reference frame")
    p.add_argument("--max-linear-vel", type=float, default=0.02,
                   help="pih: velocity cap on the force-controlled axis (m/s)")
    return p


def main():
    args = build_parser().parse_args()
    if not 1.0 <= args.rate <= 100.0:
        raise SystemExit("--rate must be 1..100 Hz "
                         "(SendCartesianMotionForce is an NRT setpoint API)")
    if args.stage in ("press", "pih") and not args.i_know_the_bias:
        raise SystemExit(
            f"{args.stage} commands a force, and a commanded force means "
            "nothing while\nthe arm reports ~24 N at rest with nothing "
            "touching it. Declare the tool\nin Flexiv Elements, confirm with "
            "`rdk_primitive_pih.py <sn> check` that the\nresting wrench is "
            "near zero, or re-run with --i-know-the-bias.\n\n"
            "`soft` needs no such flag: it commands no force, so the bias only "
            "sags it."
        )
    signal.signal(signal.SIGINT, _on_sigint)

    if not args.yes_move:
        print(f"Stage {args.stage!r} enables the arm and makes it compliant —")
        print("it will move under external force, and it will sag by")
        print("(resting force bias / stiffness) as soon as the stiffness lands.")
        print("E-stop in hand, clearance around the arm, then:")
        print(f"\n  python3 rdk_compliance_demo.py {args.robot_sn} "
              f"{args.stage} --yes-move\n")
        for k, v in sorted(vars(args).items()):
            if k not in ("robot_sn", "stage", "yes_move"):
                print(f"  {k:22s} {v}")
        return 0

    try:
        rdk, robot = connect_and_enable(args.robot_sn, args.enable_timeout)
        _STAGES[args.stage](args, rdk, robot)
    except SystemExit:
        raise
    except Exception as exc:  # noqa: BLE001
        _stop_robot(f"{type(exc).__name__}: {exc}")
        print(f"\nFAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        _stop_robot("stage finished")
    print("\nDone.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
