"""Shared plumbing for the standalone rdk_*.py hardware scripts.

Deliberately importable without sourcing the workspace: Python puts a script's
own directory on sys.path, so `import rdk_common` works from any cwd as long
as this file sits next to the script. These scripts are run straight out of the
source tree with plain python3 on the robot's host, so they must not depend on
the ament install tree.

No rclpy, no ROS. flexivrdk is imported lazily inside connect() so that -h and
the argument-validation paths work on a machine without the bindings.
"""
from __future__ import annotations

import math
import signal
import sys
import time

# Set by connect() so the signal handler and the failure paths can stop the arm
# without every script threading a robot handle through its call stack.
_ROBOT = None
_STOPPED = False
# Set once Enable() has succeeded. Until then there is nothing to stop, so an
# argument rejected during planning does not print a Stop() line it did not
# need -- and does not call Stop() on a robot that never left IDLE.
_ENGAGED = False

# operational_status() values and what to do about each. Text follows the RDK
# 1.9.3 headers (data.hpp): "Except for the first two, the other enumerators
# indicate the cause of the robot being not ready to operate." Note that plain
# Auto mode is listed as a blocking cause -- RDK needs Auto (Remote).
STATUS_ADVICE = {
    "READY": None,
    "NOT_ENABLED": "call Enable() — this script does that",
    "BOOTING": "controller still booting, wait and retry",
    "RELEASING_BRAKE": "brake release in progress, wait",
    "ESTOP_NOT_RELEASED": "release the E-stop",
    "MINOR_FAULT": "ClearFault() — this script tries that",
    "CRITICAL_FAULT": "clear the fault in Flexiv Elements; check its event log",
    "IN_REDUCED_STATE": "robot is in reduced state; clear the reduced-speed "
                        "condition in Elements",
    "IN_RECOVERY_STATE": "robot is in recovery state; run recovery in Elements "
                         "to bring joints back inside their limits",
    "IN_MANUAL_MODE": "switch the robot to Auto (Remote) mode — RDK cannot "
                      "take control in Manual mode",
    "IN_AUTO_MODE": "switch from regular Auto to Auto (REMOTE) mode — plain "
                    "Auto is not enough for RDK",
}

# Statuses no amount of waiting resolves: they need a person to change
# something on the robot, so fail fast instead of burning the enable timeout.
BLOCKING_STATUSES = (
    "IN_MANUAL_MODE", "IN_AUTO_MODE", "ESTOP_NOT_RELEASED", "CRITICAL_FAULT",
    "IN_RECOVERY_STATE",
)


# ── formatting ───────────────────────────────────────────────────────────────
def fmt(vals, prec=2, width=7):
    try:
        return "[" + " ".join(f"{float(v):{width}.{prec}f}" for v in vals) + "]"
    except TypeError:
        return repr(vals)


def norm3(vals, n=3):
    """Magnitude of the first n components (the force part of a wrench)."""
    try:
        return math.sqrt(sum(float(v) ** 2 for v in list(vals)[:n]))
    except (TypeError, ValueError):
        return float("nan")


def deg(rad):
    return [math.degrees(float(x)) for x in rad]


# ── stopping ─────────────────────────────────────────────────────────────────
def stop_robot(reason: str) -> None:
    """Stop() once, ever, and never raise out of the attempt."""
    global _STOPPED
    if _ROBOT is None or _STOPPED or not _ENGAGED:
        return
    _STOPPED = True
    print(f"\n[safety] Stop() — {reason}", flush=True)
    try:
        _ROBOT.Stop()
    except Exception as exc:  # noqa: BLE001
        # Stop() switches mode, which the controller refuses when the robot was
        # never operational (e.g. still in Manual mode). Harmless: there is
        # nothing to stop.
        print(f"[safety] Stop() declined ({exc}) — nothing was moving")


def install_sigint_handler() -> None:
    def _handler(_signum, _frame):
        stop_robot("SIGINT")
        sys.exit(130)

    signal.signal(signal.SIGINT, _handler)


# ── connect / enable ─────────────────────────────────────────────────────────
def status_name(robot) -> str:
    return str(robot.operational_status()).rsplit(".", 1)[-1]


def connect(sn: str):
    """Instantiate the robot and register it for the stop paths."""
    global _ROBOT
    import flexivrdk  # noqa: WPS433  (lazy: see module docstring)

    print(f"flexivrdk {getattr(flexivrdk, '__version__', '?')} "
          f"at {flexivrdk.__file__}")
    print(f"Connecting Robot({sn!r})")
    _ROBOT = flexivrdk.Robot(sn)
    return flexivrdk, _ROBOT


def clear_fault(robot) -> None:
    if robot.fault():
        print("fault set — ClearFault()")
        if not robot.ClearFault():
            raise RuntimeError("ClearFault() failed; clear it in Elements first")


def enable(robot, timeout_sec: float = 30.0) -> None:
    """Enable and wait for operational, failing fast on the blocking statuses.

    Enable() cannot change the robot's mode, so waiting out the timeout on a
    robot in Manual mode tells you nothing. Check first, and keep reporting the
    status while waiting on the transient ones.
    """
    clear_fault(robot)

    status = status_name(robot)
    if status in BLOCKING_STATUSES:
        raise RuntimeError(
            f"robot is {status} and Enable() cannot change that — "
            f"{STATUS_ADVICE.get(status, 'see Flexiv Elements')}"
        )

    print(f"status {status} — Enable()")
    robot.Enable()
    t0 = time.monotonic()
    last_report = 0.0
    while not robot.operational():
        status = status_name(robot)
        if status in BLOCKING_STATUSES:
            raise RuntimeError(
                f"robot became {status} while enabling — "
                f"{STATUS_ADVICE.get(status, 'see Flexiv Elements')}"
            )
        waited = time.monotonic() - t0
        if waited - last_report >= 2.0:
            last_report = waited
            print(f"  waiting for operational: {status} ({waited:.0f}s)",
                  flush=True)
        if waited >= timeout_sec:
            raise TimeoutError(
                f"not operational after {timeout_sec:.0f}s (status {status}: "
                f"{STATUS_ADVICE.get(status, 'unknown cause')})"
            )
        time.sleep(0.5)
    global _ENGAGED
    _ENGAGED = True
    print("operational")


def lock_external_axes(robot, locked: bool = True) -> int:
    """Lock/unlock the waist. Requires IDLE, so call before SwitchMode.

    Returns the number of external axes, 0 if the robot has none.
    """
    ext_dof = int(getattr(robot.info(), "DoF_e", 0) or 0)
    if ext_dof > 0:
        robot.LockExternalAxes(bool(locked))
        print(f"external axes ({ext_dof}) "
              f"{'LOCKED' if locked else 'unlocked'}")
    return ext_dof


# ── measurement ──────────────────────────────────────────────────────────────
def resting_wrench(robot, seconds: float = 3.0):
    """Mean and peak |F| of ext_wrench_in_tcp over a window.

    With has_FT_sensor False this is the joint-torque estimate, which on this
    arm reads ~24 N at rest because the tool is not declared. Every force
    number downstream inherits that offset.
    """
    n, acc, peak = 0, [0.0] * 6, 0.0
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        w = [float(x) for x in robot.states().ext_wrench_in_tcp]
        acc = [a + b for a, b in zip(acc, w)]
        peak = max(peak, norm3(w))
        n += 1
        time.sleep(0.002)
    return [a / max(n, 1) for a in acc], peak


# ── controller event log ─────────────────────────────────────────────────────
def event_key(event):
    """Identity for one controller event. RobotEvent.id is an error code, not
    unique, so pair it with the timestamp and text."""
    return (repr(getattr(event, "timestamp", None)),
            getattr(event, "id", None),
            getattr(event, "description", ""))


def event_keys(robot):
    try:
        return {event_key(e) for e in robot.event_log()}
    except Exception:  # noqa: BLE001
        return set()


def print_new_events(robot, before, only_bad=True):
    """Print controller events that appeared since the `before` snapshot.

    The controller reports things RDK's Python calls do not raise on -- an
    unlicensed primitive is logged as event 303009 while ExecutePrimitive
    returns normally -- so this is the only way to see why nothing happened.
    """
    try:
        events = robot.event_log()
    except Exception as exc:  # noqa: BLE001
        print(f"[events] event_log() unavailable: {exc}")
        return []
    new = [e for e in events if event_key(e) not in before]
    shown = [e for e in new
             if not only_bad
             or str(getattr(e, "level", "")).rsplit(".", 1)[-1]
             in ("ERROR", "CRITICAL")]
    for e in shown:
        level = str(getattr(e, "level", "?")).rsplit(".", 1)[-1]
        print(f"[events] {level} [{getattr(e, 'id', '?')}] "
              f"{getattr(e, 'description', '')}")
        for field in ("probable_causes", "recommended_actions"):
            text = getattr(e, field, "")
            if text:
                print(f"           {field}: {text}")
    return shown


# ── the --yes-move gate ──────────────────────────────────────────────────────
def gate_move(args, script_name, warning_lines=()):
    """Print what a stage would do and return False if --yes-move is absent."""
    if getattr(args, "yes_move", False):
        return True
    print(f"Stage {args.stage!r} enables the arm and can move it.")
    for line in warning_lines:
        print(line)
    print("Re-run with --yes-move once the E-stop is in your hand and the arm")
    print("has clearance:")
    print(f"\n  python3 {script_name} {args.robot_sn} {args.stage} --yes-move\n")
    print("Argument values this stage would use:")
    for k, v in sorted(vars(args).items()):
        if k not in ("robot_sn", "stage", "yes_move"):
            print(f"  {k:22s} {v}")
    return False
