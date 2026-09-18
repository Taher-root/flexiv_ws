#!/usr/bin/env python3
"""Staged runner for the Flexiv Adaptive Assembly peg-in-hole primitives.

Unlike rdk_diag.py and rdk_compliance_probe.py, this script CAN MOVE THE ARM.
Every stage that moves is behind an explicit --yes-move flag; without it the
stage prints the exact primitive name and parameter dict it would send, then
exits. Read the stage descriptions below before using --yes-move.

Verified against the installed bindings (flexivrdk 1.9.0, Rizon4, robot
software v3.11), not against documentation for another release:

    ExecutePrimitive(primitive_name: str,
                     input_params: dict[str, int|float|str|JPos|Coord|list[...]],
                     block_until_started: bool = True) -> None
    primitive_states() -> dict[str, int|float|str|JPos|Coord|list[...]]

Booleans in primitive parameters are ints 1/0 (RDK header note). Primitive
parameters do NOT universally use SI units — every value below is taken from
the primitive's own documentation page for robot software v3.11.

STAGES
  check    No motion, no enable. info(), license, K_x_nom, TCP pose, and the
           resting wrench with peak-hold. Run this first, every session.
  bias     No motion, no enable. Measures the standing wrench over --seconds
           and writes it to --bias-file. This quantifies the tool-gravity
           offset; it does NOT fix it (see THE BIAS PROBLEM).
  zero     Enables the arm and runs the ZeroFTSensor primitive. Static data
           collection, the arm should not travel. The one stage worth trying
           before anything moves, because it is the cheapest test that
           primitive execution works at all on this license.
  search   SearchHole. The tool must already be in contact with the surface
           near the hole. Spiral search at a held contact force.
  checkpih CheckPiH. Probes four directions to decide whether the peg is
           seated. Moves up to --search-range in each direction.
  insert   InsertComp. Drives along --insert-axis until the external force
           reaches --max-contact-force, complying on the other axes.

THE BIAS PROBLEM (read this before running any force primitive)
  info().has_FT_sensor is False on this arm: there is no wrist force/torque
  sensor, so every force number comes from the joint torque sensors through
  the Jacobian. Measured at rest, untouched, the arm reports roughly
  [-2.3, -23.9, 6.2] N -- about 24 N of force that is not there. That is
  almost certainly the gripper's mass, uncompensated because the tool is not
  declared to the controller.

  This matters because the primitives are parameterised in newtons against
  the controller's own estimate. Tell SearchHole to hold contactForce=5 while
  the controller already believes it is pushing 24 N and it will back off
  rather than press, then report lostContact. No parameter choice fixes that.

  The real fix is to declare the tool (mass, centre of mass, inertia) in
  Flexiv Elements so the controller's gravity compensation is correct, after
  which the resting wrench should read near zero -- check with the `check`
  stage. The `zero` stage is worth one attempt as a shortcut, but
  ZeroFTSensor is documented against an F/T sensor this arm does not have.

  The --bias-file the `bias` stage writes is used only to make THIS script's
  printed force readings honest. It cannot change what the controller
  believes, so it cannot make a force threshold inside a primitive correct.

SAFETY
  - E-stop in hand. These primitives are compliant: the arm yields, and it
    also pushes.
  - Lock the waist (default) so a primitive cannot drive the external axes.
  - Every primitive gets enableMaxWrench + maxContactWrench set, and the
    script Stop()s on Ctrl-C, on --max-seconds, and on any exception.
  - Start in Manual mode at reduced speed if you can. The time factors in
    these primitives roughly double in Manual mode, which is the safe
    direction.

Usage:
    python3 rdk_primitive_pih.py <robot_sn> check
    python3 rdk_primitive_pih.py <robot_sn> bias --seconds 10
    python3 rdk_primitive_pih.py <robot_sn> zero --yes-move
    python3 rdk_primitive_pih.py <robot_sn> search            # prints, no motion
    python3 rdk_primitive_pih.py <robot_sn> search --yes-move
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time

# Stop() on the way out of anything, including Ctrl-C.
_ROBOT = None
_STOPPED = False

# primitive_states() key aliases: the v3.11 SearchHole page lists the state as
# "pushDis" in its table and "pushDistance" in its default transition
# condition. Accept either rather than guessing which the controller uses.
_KEY_ALIASES = {
    "pushDis": ("pushDis", "pushDistance"),
    "pushDistance": ("pushDistance", "pushDis"),
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
        print(f"[safety] Stop() raised: {exc}", flush=True)


def _on_sigint(_signum, _frame):
    _stop_robot("SIGINT")
    sys.exit(130)


def _fmt(vals, prec=2):
    try:
        return "[" + " ".join(f"{float(v):7.{prec}f}" for v in vals) + "]"
    except TypeError:
        return repr(vals)


def _norm3(vals):
    try:
        return math.sqrt(sum(float(v) ** 2 for v in list(vals)[:3]))
    except (TypeError, ValueError):
        return float("nan")


def _wrench(robot, bias=None):
    """External wrench at the TCP, optionally with a measured bias removed."""
    w = [float(x) for x in robot.states().ext_wrench_in_tcp]
    if bias:
        w = [a - b for a, b in zip(w, bias)]
    return w


def _state(states: dict, key: str, default=None):
    for k in _KEY_ALIASES.get(key, (key,)):
        if k in states:
            return states[k]
    return default


# ── connect / enable ─────────────────────────────────────────────────────────
def connect(sn: str):
    global _ROBOT
    import flexivrdk

    print(f"flexivrdk {getattr(flexivrdk, '__version__', '?')} at {flexivrdk.__file__}")
    print(f"Connecting Robot({sn!r})")
    _ROBOT = flexivrdk.Robot(sn)
    return flexivrdk, _ROBOT


def enable_for_primitives(robot, rdk, lock_waist: bool, enable_timeout: float):
    """Clear fault, enable, lock external axes, switch to primitive execution."""
    if robot.fault():
        print("fault set — ClearFault()")
        if not robot.ClearFault():
            raise RuntimeError("ClearFault() failed; clear it in Elements first")

    print("Enable() — E-stop released, motion bar in Auto Remote (or Manual)")
    robot.Enable()
    deadline = time.monotonic() + enable_timeout
    while not robot.operational():
        if time.monotonic() >= deadline:
            raise TimeoutError(
                f"not operational after {enable_timeout:.0f}s "
                f"(status: {robot.operational_status()})"
            )
        time.sleep(0.5)
    print("operational")

    # LockExternalAxes requires IDLE, so it has to happen before SwitchMode.
    # Locked by default: nothing here should be driving the waist.
    ext_dof = int(getattr(robot.info(), "DoF_e", 0) or 0)
    if ext_dof > 0:
        robot.LockExternalAxes(bool(lock_waist))
        print(f"external axes ({ext_dof}) {'LOCKED' if lock_waist else 'unlocked'}")

    robot.SwitchMode(rdk.Mode.NRT_PRIMITIVE_EXECUTION)
    print(f"mode: {robot.mode()}")


def run_primitive(robot, name, params, done_keys, max_seconds, bias=None,
                  poll_sec=0.25):
    """Execute one primitive and stream its states until a done key is reached.

    done_keys entries are either a state key, satisfied when that state is
    truthy, or a (key, predicate, description) triple for states that carry a
    measurement rather than a flag -- SearchHole's pushDis is a distance in
    metres whose documented transition is > 0.005, not "nonzero".

    Returns the final states dict. Stop()s and raises on timeout or fault.
    """
    print()
    print("=" * 72)
    print(f"ExecutePrimitive({name!r}, {params!r})")
    print("=" * 72)
    robot.ExecutePrimitive(name, params)

    t0 = time.monotonic()
    states: dict = {}
    while True:
        states = dict(robot.primitive_states())
        elapsed = time.monotonic() - t0

        w = _wrench(robot, bias)
        shown = {k: states[k] for k in sorted(states) if k not in ("timePeriod",)}
        print(f"t={elapsed:5.1f}s |F|={_norm3(w):6.2f}N {_fmt(w)}  {shown}",
              flush=True)

        for entry in done_keys:
            if isinstance(entry, tuple):
                key, predicate, description = entry
            else:
                key, predicate, description = entry, bool, "is true"
            value = _state(states, key)
            if value is not None and predicate(value):
                print(f"\ndone: primitive state {key!r} {description} after "
                      f"{elapsed:.1f}s")
                return states

        if robot.fault():
            _stop_robot(f"fault raised during {name}")
            raise RuntimeError(f"{name}: robot faulted — check Elements event log")
        if elapsed >= max_seconds:
            _stop_robot(f"{name} exceeded --max-seconds")
            raise TimeoutError(
                f"{name}: none of {done_keys} became true in {max_seconds:.0f}s; "
                f"last states: {states}"
            )
        time.sleep(poll_sec)


# ── stages ───────────────────────────────────────────────────────────────────
def stage_check(args, rdk, robot):
    """No motion, no enable."""
    info = robot.info()
    print()
    print("=" * 72)
    print("ROBOT")
    print("=" * 72)
    for field in ("serial_num", "model_name", "software_ver", "license_type",
                  "DoF", "DoF_m", "DoF_e", "has_FT_sensor"):
        print(f"  {field:16s} {getattr(info, field, '<absent>')}")
    print(f"  {'K_x_nom':16s} {_fmt(info.K_x_nom, 1)}")
    print(f"  {'mode':16s} {robot.mode()}")
    print(f"  {'operational':16s} {robot.operational()}  "
          f"status={robot.operational_status()}")
    print(f"  {'fault':16s} {robot.fault()}")

    st = robot.states()
    print(f"  {'tcp_pose':16s} {_fmt(st.tcp_pose, 4)}")
    print(f"  {'q (rad)':16s} {_fmt(st.q, 4)}")

    print()
    print(f"Resting wrench over {args.seconds:.0f}s — DO NOT TOUCH THE ARM")
    mean, peak = _sample_wrench(robot, args.seconds)
    print(f"  mean {_fmt(mean)}  |F|={_norm3(mean):.2f} N")
    print(f"  peak |F| {peak:.2f} N")
    print()
    if _norm3(mean) > 3.0:
        print(f"  {_norm3(mean):.1f} N of force the arm is not feeling. Declare the")
        print("  tool/payload in Flexiv Elements before trusting any force")
        print("  threshold in a primitive. See THE BIAS PROBLEM in this file.")
    else:
        print("  Resting wrench is small — gravity compensation looks right.")


def _sample_wrench(robot, seconds):
    n, acc, peak = 0, [0.0] * 6, 0.0
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        w = _wrench(robot)
        acc = [a + b for a, b in zip(acc, w)]
        peak = max(peak, _norm3(w))
        n += 1
        time.sleep(0.002)
    return ([a / max(n, 1) for a in acc], peak)


def stage_bias(args, rdk, robot):
    """No motion, no enable. Measure and store the resting wrench."""
    print(f"\nMeasuring resting wrench for {args.seconds:.0f}s — "
          "DO NOT TOUCH THE ARM")
    mean, peak = _sample_wrench(robot, args.seconds)
    print(f"  mean {_fmt(mean, 3)}")
    print(f"  |F| mean {_norm3(mean):.3f} N   peak {peak:.3f} N")
    payload = {
        "robot_sn": args.robot_sn,
        "measured_at": time.time(),
        "seconds": args.seconds,
        "ext_wrench_in_tcp_bias": mean,
        "tcp_pose": [float(x) for x in robot.states().tcp_pose],
        "note": (
            "Bias is pose-dependent: tool gravity projects differently at a "
            "different TCP orientation. Re-measure in the pose you will work "
            "in. This only corrects this script's printed numbers, never what "
            "the controller believes."
        ),
    }
    with open(args.bias_file, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2)
    print(f"\nwrote {args.bias_file}")


def stage_zero(args, rdk, robot):
    """Static F/T zeroing. Enables the arm; should not travel."""
    info = robot.info()
    if not getattr(info, "has_FT_sensor", False):
        print()
        print("NOTE: info().has_FT_sensor is False. ZeroFTSensor is documented")
        print("      against a wrist F/T sensor this arm does not have, so this")
        print("      may be rejected or may do nothing. Trying it anyway is")
        print("      still the cheapest test that primitive execution works on")
        print("      this license — a rejection tells you the license, a fault")
        print("      tells you the primitive, success tells you both.")
    enable_for_primitives(robot, rdk, args.lock_waist, args.enable_timeout)
    run_primitive(
        robot, "ZeroFTSensor",
        {
            "dataCollectTime": 0.2,
            "enableStaticCheck": 0,
            "calibExtraPayload": 1 if args.calib_payload else 0,
        },
        done_keys=["terminated"],
        max_seconds=args.max_seconds,
    )
    print("\nResting wrench after zeroing:")
    mean, _ = _sample_wrench(robot, 3.0)
    print(f"  mean {_fmt(mean)}  |F|={_norm3(mean):.2f} N")


def stage_search(args, rdk, robot):
    """SearchHole: spiral search at a held contact force."""
    params = {
        "contactAxis": args.contact_axis,          # VEC_3i, TCP frame
        "contactForce": args.contact_force,        # N, 5 in [1..20]
        "searchAxis": [float(x) for x in args.search_axis],
        "searchPattern": args.search_pattern,      # SPIRAL | ZIGZAG
        "spiralRadius": args.spiral_radius,        # m, in [0.001..0.015]
        "startDensity": args.start_density,        # in [1..5]
        "timeFactor": args.time_factor,            # in [1..10]
        "wiggleRange": args.wiggle_range,          # deg, in [0..90]
        "wigglePeriod": args.wiggle_period,        # s, in [0.2..30]
        "enableMaxWrench": [1, 1, 1, 1, 1, 1],
        "maxContactWrench": args.max_contact_wrench,
        "searchImmed": 1 if args.search_immed else 0,
        "searchStiffRatio": args.search_stiff_ratio,
        "maxVelForceDir": args.max_vel_force_dir,  # m/s, in [0.005..0.5]
    }
    if args.search_pattern == "ZIGZAG":
        params["zigzagLength"] = args.zigzag_length
        params["zigzagWidth"] = args.zigzag_width
        params.pop("spiralRadius")

    enable_for_primitives(robot, rdk, args.lock_waist, args.enable_timeout)
    # pushDis crossing the documented 0.005 m is the success signal; terminated
    # and lostContact are the two ways it ends without finding anything.
    states = run_primitive(
        robot, "SearchHole", params,
        done_keys=[
            ("pushDis", lambda v: float(v) > args.push_threshold,
             f"exceeded {args.push_threshold} m"),
            "terminated",
            "lostContact",
        ],
        max_seconds=args.max_seconds, bias=_load_bias(args),
    )
    push = _state(states, "pushDis", 0.0) or 0.0
    if _state(states, "lostContact"):
        print("\nlostContact: all axes read under 1 N for 1.5 s. The tool is "
              "not touching the surface, or the resting bias is being read as "
              "contact force.")
    elif float(push) > args.push_threshold:
        print(f"\nhole found: pushDis {float(push) * 1000:.1f} mm "
              f"(threshold {args.push_threshold * 1000:.1f} mm)")
    else:
        print(f"\nterminated without finding the hole: pushDis "
              f"{float(push) * 1000:.1f} mm, "
              f"searchResisForce {_state(states, 'searchResisForce')}")


def stage_checkpih(args, rdk, robot):
    """CheckPiH: probe four directions to decide whether the peg is seated."""
    params = {
        "contactAxis": [float(x) for x in args.contact_axis],
        "searchAxis": [float(x) for x in args.search_axis],
        "searchRange": args.search_range,   # m, in [0.001..0.1]
        "searchForce": args.search_force,   # N, in [1..20]
        "searchVel": args.search_vel,       # m/s, in [0.001..0.1]
        "linearSearchOnly": 1 if args.linear_search_only else 0,
        "returnToInitPose": 1 if args.return_to_init else 0,
    }
    enable_for_primitives(robot, rdk, args.lock_waist, args.enable_timeout)
    states = run_primitive(
        robot, "CheckPiH", params,
        done_keys=["checkComplete", "terminated"],
        max_seconds=args.max_seconds, bias=_load_bias(args),
    )
    print(f"\npegIsInHole: {_state(states, 'pegIsInHole')}")


def stage_insert(args, rdk, robot):
    """InsertComp: drive along one axis, comply on the others."""
    params = {
        "insertAxis": args.insert_axis,             # X -X Y -Y Z -Z, required
        "compAxis": args.comp_axis,                 # VEC_6i
        "maxContactForce": args.max_contact_force,  # N, in [1..120]
        "deadbandScale": args.deadband_scale,       # in [0..100]
        "insertVel": args.insert_vel,               # m/s, in [0.001..0.1]
        "compVelScale": args.comp_vel_scale,        # in [10..100]
    }
    enable_for_primitives(robot, rdk, args.lock_waist, args.enable_timeout)
    # Transitions on isMoving == 0, so wait for it to go false rather than true.
    print()
    print("=" * 72)
    print(f"ExecutePrimitive('InsertComp', {params!r})")
    print("=" * 72)
    robot.ExecutePrimitive("InsertComp", params)
    bias = _load_bias(args)
    t0 = time.monotonic()
    saw_motion = False
    while True:
        states = dict(robot.primitive_states())
        elapsed = time.monotonic() - t0
        moving = bool(_state(states, "isMoving", 0))
        saw_motion = saw_motion or moving
        w = _wrench(robot, bias)
        print(f"t={elapsed:5.1f}s |F|={_norm3(w):6.2f}N insertDis="
              f"{_state(states, 'insertDis')} isMoving={moving}", flush=True)
        # isMoving can still read 0 in the first moments while the motion
        # ramps up, so a falling edge is the signal; a 0 that outlasts the
        # grace period is accepted as "never started" rather than hanging on
        # to the watchdog.
        if not moving and (saw_motion or elapsed > args.insert_grace):
            what = "insertion stopped" if saw_motion else (
                f"isMoving never went true in {args.insert_grace:.0f}s")
            print(f"\n{what} after {elapsed:.1f}s, "
                  f"insertDis={_state(states, 'insertDis')}")
            return states
        if _state(states, "terminated"):
            print(f"\nterminated after {elapsed:.1f}s, "
                  f"insertDis={_state(states, 'insertDis')}")
            return states
        if robot.fault():
            _stop_robot("fault raised during InsertComp")
            raise RuntimeError("InsertComp: robot faulted")
        if elapsed >= args.max_seconds:
            _stop_robot("InsertComp exceeded --max-seconds")
            raise TimeoutError(f"InsertComp: still running; last {states}")
        time.sleep(0.25)


def _load_bias(args):
    if not args.use_bias:
        return None
    try:
        with open(args.bias_file, encoding="utf-8") as fh:
            bias = json.load(fh)["ext_wrench_in_tcp_bias"]
    except (OSError, KeyError, ValueError) as exc:
        print(f"[bias] ignoring {args.bias_file}: {exc}")
        return None
    print(f"[bias] subtracting {_fmt(bias)} from printed wrenches only — "
          "the controller still uses its own uncorrected estimate")
    return bias


_MOVING_STAGES = {"zero", "search", "checkpih", "insert"}
_STAGES = {
    "check": stage_check,
    "bias": stage_bias,
    "zero": stage_zero,
    "search": stage_search,
    "checkpih": stage_checkpih,
    "insert": stage_insert,
}


def build_parser():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("robot_sn")
    p.add_argument("stage", choices=sorted(_STAGES))
    p.add_argument("--yes-move", action="store_true",
                   help="required for zero/search/checkpih/insert; without it "
                        "the stage prints what it would send and exits")
    p.add_argument("--seconds", type=float, default=5.0,
                   help="sampling window for check/bias")
    p.add_argument("--max-seconds", type=float, default=60.0,
                   help="watchdog: Stop() the arm if a primitive runs longer")
    p.add_argument("--enable-timeout", type=float, default=30.0)
    p.add_argument("--no-lock-waist", dest="lock_waist", action="store_false",
                   help="allow primitives to drive the external axes (don't)")
    p.add_argument("--bias-file",
                   default=os.path.expanduser("~/.ros/aico2_wrench_bias.json"))
    p.add_argument("--use-bias", action="store_true",
                   help="subtract the measured bias from PRINTED wrenches")
    p.add_argument("--calib-payload", action="store_true",
                   help="zero stage: set ZeroFTSensor calibExtraPayload=1")

    g = p.add_argument_group("SearchHole")
    g.add_argument("--contact-axis", type=int, nargs=3, default=[0, 0, 1])
    g.add_argument("--contact-force", type=float, default=5.0)
    g.add_argument("--search-axis", type=float, nargs=3, default=[1.0, 0.0, 0.0])
    g.add_argument("--search-pattern", choices=["SPIRAL", "ZIGZAG"],
                   default="SPIRAL")
    g.add_argument("--spiral-radius", type=float, default=0.008)
    g.add_argument("--zigzag-length", type=float, default=0.02)
    g.add_argument("--zigzag-width", type=float, default=0.02)
    g.add_argument("--start-density", type=int, default=2)
    g.add_argument("--time-factor", type=int, default=4,
                   help="higher is slower; 4 rather than the documented 2")
    g.add_argument("--wiggle-range", type=float, default=0.0)
    g.add_argument("--wiggle-period", type=float, default=0.3)
    g.add_argument("--search-stiff-ratio", type=float, default=1.0)
    g.add_argument("--max-vel-force-dir", type=float, default=0.02,
                   help="m/s along the force axis; documented default 0.1")
    g.add_argument("--no-search-immed", dest="search_immed",
                   action="store_false",
                   help="wait for contactForce before searching (makes "
                        "forceDrop meaningful)")
    g.add_argument("--push-threshold", type=float, default=0.005,
                   help="pushDis (m) that counts as finding the hole; the "
                        "primitive's own documented transition is 0.005")
    g.add_argument("--max-contact-wrench", type=float, nargs=6,
                   default=[50.0, 50.0, 50.0, 15.0, 15.0, 15.0],
                   help="Fx Fy Fz Mx My Mz; documented default is "
                        "150/150/150/40/40/40, floor is 5/5/5/1/1/1")

    g = p.add_argument_group("CheckPiH")
    g.add_argument("--search-range", type=float, default=0.005)
    g.add_argument("--search-force", type=float, default=3.0)
    g.add_argument("--search-vel", type=float, default=0.005)
    g.add_argument("--linear-search-only", action="store_true")
    g.add_argument("--no-return-to-init", dest="return_to_init",
                   action="store_false")

    g = p.add_argument_group("InsertComp")
    g.add_argument("--insert-axis", choices=["X", "-X", "Y", "-Y", "Z", "-Z"],
                   default="Z")
    g.add_argument("--comp-axis", type=int, nargs=6, default=[1, 1, 0, 1, 1, 0],
                   help="comply on X Y Rx Ry, hold the insertion axis")
    g.add_argument("--max-contact-force", type=float, default=5.0)
    g.add_argument("--deadband-scale", type=float, default=50.0)
    g.add_argument("--insert-vel", type=float, default=0.005)
    g.add_argument("--comp-vel-scale", type=float, default=20.0)
    g.add_argument("--insert-grace", type=float, default=3.0,
                   help="seconds to allow isMoving to go true before treating "
                        "a 0 as 'never started'")
    return p


def main():
    args = build_parser().parse_args()
    signal.signal(signal.SIGINT, _on_sigint)

    if args.stage in _MOVING_STAGES and not args.yes_move:
        print(f"Stage {args.stage!r} enables the arm and can move it.")
        print("Re-run with --yes-move once the E-stop is in your hand, the arm")
        print("has clearance, and you have read THE BIAS PROBLEM in this file:")
        print()
        print(f"  python3 {os.path.basename(sys.argv[0])} {args.robot_sn} "
              f"{args.stage} --yes-move")
        print()
        print("Argument values this stage would use:")
        for k, v in sorted(vars(args).items()):
            if k not in ("robot_sn", "stage", "yes_move"):
                print(f"  {k:22s} {v}")
        return 0

    rdk, robot = connect(args.robot_sn)
    try:
        _STAGES[args.stage](args, rdk, robot)
    except Exception as exc:  # noqa: BLE001
        _stop_robot(f"{type(exc).__name__}: {exc}")
        print(f"\nFAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        if args.stage in _MOVING_STAGES:
            _stop_robot("stage finished")
    print("\nDone.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
