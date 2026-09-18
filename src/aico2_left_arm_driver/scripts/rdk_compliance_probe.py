#!/usr/bin/env python3
"""Read-only probe for force sensing and compliance/impedance tuning.

SAFETY: this script never calls Enable(), SwitchMode(), Stop(), or any Send*
method. It only constructs Robot(), reads info()/states(), and introspects the
Python bindings. It cannot move the arm. The arm does not need to be enabled —
states() streams regardless (verified: rdk_diag.py read q/dq/tau with the arm
never enabled).

Answers two questions:

  1. What can the arm sense when you push the end effector?
     Streams every force/torque field live with a peak-hold, so a push shows
     up even if you can't read every line as it scrolls.

  2. What API exists to make it comply?
     Introspects flexivrdk.Robot for impedance/stiffness/force-control methods
     and prints their real pybind11 signatures. This is authoritative for the
     installed version, unlike documentation for some other release.

Usage:
    python3 rdk_compliance_probe.py <robot_sn>              # introspect + 20s stream
    python3 rdk_compliance_probe.py <robot_sn> --api-only   # introspect only
    python3 rdk_compliance_probe.py <robot_sn> --seconds 60
"""
import argparse
import sys
import time

# Anything plausibly related to compliance, admittance or force control.
API_KEYWORDS = (
    "impedance", "stiffness", "compliance", "admittance", "force", "wrench",
    "contact", "nullspace", "null_space", "passive", "damping", "insert",
    "primitive", "cartesian", "torque", "limit", "payload", "tool",
)

# states() fields that carry force/torque information.
FORCE_FIELDS = (
    "ext_wrench_in_tcp",
    "ext_wrench_in_tcp_raw",
    "ext_wrench_in_world",
    "ext_wrench_in_world_raw",
    "ft_sensor_raw",
    "tau_ext",
    "tau_interact",
)


def _fmt(vals, width=8, prec=3):
    try:
        return "[" + " ".join(f"{float(v):{width}.{prec}f}" for v in vals) + "]"
    except TypeError:
        return repr(vals)


def _norm(vals, n=3):
    """Magnitude of the first n components (force part of a wrench)."""
    try:
        return sum(float(v) ** 2 for v in list(vals)[:n]) ** 0.5
    except (TypeError, ValueError):
        return 0.0


def dump_api(rdk, robot):
    print("=" * 72)
    print("CONTROL MODES available (flexivrdk.Mode)")
    print("=" * 72)
    for name in sorted(n for n in dir(rdk.Mode) if not n.startswith("_")):
        print(f"  Mode.{name}")

    print()
    print("=" * 72)
    print("Robot METHODS matching compliance/force keywords")
    print("=" * 72)
    names = sorted(n for n in dir(robot) if not n.startswith("_"))
    matched = [n for n in names if any(k in n.lower() for k in API_KEYWORDS)]
    for n in matched:
        try:
            attr = getattr(robot, n)
        except Exception as exc:  # noqa: BLE001
            print(f"\n--- {n} --- (could not access: {exc})")
            continue
        doc = (getattr(attr, "__doc__", None) or "").strip()
        print(f"\n--- {n} ---")
        if doc:
            # pybind11 puts the real signature on the first line(s).
            for line in doc.splitlines():
                print(f"    {line}")
        else:
            print(f"    (no docstring; type={type(attr).__name__})")

    print()
    print("=" * 72)
    print(f"Robot attributes NOT matched by those keywords ({len(names) - len(matched)})")
    print("=" * 72)
    print("  " + ", ".join(n for n in names if n not in matched))

    print()
    print("=" * 72)
    print("info() fields — look for nominal stiffness (K_x_nom / K_q_nom) and limits")
    print("=" * 72)
    info = robot.info()
    for n in sorted(n for n in dir(info) if not n.startswith("_")):
        try:
            v = getattr(info, n)
        except Exception as exc:  # noqa: BLE001
            print(f"  {n}: <error: {exc}>")
            continue
        if callable(v):
            continue
        try:
            print(f"  {n}: {_fmt(v) if hasattr(v, '__len__') and not isinstance(v, str) else v}")
        except Exception:  # noqa: BLE001
            print(f"  {n}: {v!r}")


def stream_forces(robot, seconds):
    print()
    print("=" * 72)
    print(f"LIVE FORCE/TORQUE — {seconds}s. PUSH THE END EFFECTOR NOW.")
    print("=" * 72)

    st = robot.states()
    present = [f for f in FORCE_FIELDS if hasattr(st, f)]
    missing = [f for f in FORCE_FIELDS if not hasattr(st, f)]
    print(f"present: {present}")
    if missing:
        print(f"absent on this firmware: {missing}")
    print()

    peaks = {f: 0.0 for f in present}
    t0 = time.monotonic()
    last_print = 0.0
    while time.monotonic() - t0 < seconds:
        st = robot.states()
        for f in present:
            peaks[f] = max(peaks[f], _norm(getattr(st, f)))
        now = time.monotonic() - t0
        if now - last_print >= 0.5:
            last_print = now
            w = getattr(st, "ext_wrench_in_tcp", None)
            tau_ext = getattr(st, "tau_ext", None)
            line = f"t={now:5.1f}s"
            if w is not None:
                line += f"  |F_tcp|={_norm(w):6.2f} N  wrench={_fmt(w, 7, 2)}"
            if tau_ext is not None:
                line += f"  |tau_ext|max={max(abs(float(x)) for x in tau_ext):6.2f}"
            print(line)

    print()
    print("PEAK magnitudes over the run (force part, first 3 components):")
    for f in present:
        print(f"  {f:26s} peak={peaks[f]:8.3f}")
    print()
    print("If these stayed ~0 while you pushed, the arm is not sensing the push —")
    print("check tool/payload configuration in Flexiv Elements before tuning anything.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("robot_sn")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--api-only", action="store_true")
    args = ap.parse_args()

    import flexivrdk
    print(f"flexivrdk {getattr(flexivrdk, '__version__', '?')} at {flexivrdk.__file__}")
    print(f"Connecting Robot({args.robot_sn!r}) — read-only, no Enable/SwitchMode/Send\n")
    robot = flexivrdk.Robot(args.robot_sn)

    dump_api(flexivrdk, robot)
    if not args.api_only:
        stream_forces(robot, args.seconds)

    print("\nDone. No commands were sent and no mode was changed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
