#!/usr/bin/env python3
"""Read-only RDK diagnostic. Never calls Enable(), Stop(), SwitchMode(), or any
command method — only Robot(), info(), states(). Safe to run against a live
arm at any time; it cannot move anything.

Usage:
    python3 rdk_diag.py <robot_sn>

Gathers what offset_estimator.py and waist_driver_node.py need from real
hardware:
  - flexivrdk version actually installed
  - RobotStates.timestamp cadence at 1kHz polling
  - info().DoF / DoF_m / DoF_e (whether q[0:2] really is the waist)
  - states().q / .dq / .tau lengths and a live sample

Confirmed 2026-09-17 against flexivrdk 1.9.0: RobotStates.timestamp is a
(sec, nanosec) int tuple, not a scalar — the same shape as a ROS Time
message. This script and offset_estimator.device_time_seconds() both decode
it that way now.
"""
import statistics
import sys
import time


def _device_seconds(raw_timestamp):
    """(sec, nanosec) int tuple -> float seconds. Mirrors
    offset_estimator.device_time_seconds(); kept inline so this script has
    no dependency on the ROS package and can run with bare flexivrdk."""
    sec, nanosec = raw_timestamp
    return float(sec) + float(nanosec) * 1e-9


def main():
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <robot_sn>")
        sys.exit(1)
    sn = sys.argv[1]

    import flexivrdk
    print(f"flexivrdk file: {flexivrdk.__file__}")
    print(f"flexivrdk version attr: {getattr(flexivrdk, '__version__', 'unknown')}")

    print(f"\nConnecting Robot({sn!r}) (read-only: no Enable/Stop/SwitchMode)...")
    robot = flexivrdk.Robot(sn)

    info = robot.info()
    print(f"info(): DoF={info.DoF} DoF_m={info.DoF_m} DoF_e={info.DoF_e}")

    st = robot.states()
    public_attrs = [a for a in dir(st) if not a.startswith("_")]
    print(f"\nstates() public attrs: {public_attrs}")
    print(f"q   len={len(st.q)}   values={list(st.q)}")
    print(f"dq  len={len(st.dq)}  values={list(st.dq)}")
    print(f"tau len={len(st.tau)} values={list(st.tau)}")

    ts = st.timestamp
    print(f"\ntimestamp: type={type(ts).__name__} repr={ts!r}")
    now_wall = time.time()
    print(f"host time.time() right now: {now_wall!r}")
    if isinstance(ts, tuple) and len(ts) == 2:
        ts_sec = _device_seconds(ts)
        print(f"decoded as (sec, nanosec): {ts[0]} s + {ts[1]} ns -> {ts_sec!r} s")
        print(f"host - device offset (seconds): {now_wall - ts_sec!r}")
    else:
        print("UNEXPECTED shape (not a 2-tuple) — decode manually before trusting "
              "device_time_seconds().")

    print("\nPolling for 3s to measure timestamp cadence (no sleep, tight loop)...")
    deltas = []
    t_start = time.monotonic()
    last_ts = None
    polls = 0
    while time.monotonic() - t_start < 3.0:
        s = robot.states()
        cur = s.timestamp
        if last_ts is not None and cur != last_ts:
            deltas.append(_device_seconds(cur) - _device_seconds(last_ts))
        last_ts = cur
        polls += 1
    print(f"polls={polls}  distinct_timestamp_transitions={len(deltas)}")
    if deltas:
        print(f"delta (seconds) median={statistics.median(deltas)!r} "
              f"min={min(deltas)!r} max={max(deltas)!r}")
        print("Expect ~0.001 (1 kHz) per joint_state_architecture.md sec 1.")

    print("\nDone. No commands were sent to the robot.")


if __name__ == "__main__":
    main()
