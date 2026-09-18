#!/usr/bin/env python3
"""Read-only RDK diagnostic. Never calls Enable(), Stop(), SwitchMode(), or any
command method — only Robot(), info(), states(). Safe to run against a live
arm at any time; it cannot move anything.

Usage:
    python3 rdk_diag.py <robot_sn>

Gathers what offset_estimator.py, ros_time.py and waist_driver_node.py assume
but have never verified against real hardware:
  - flexivrdk version actually installed
  - RobotStates.timestamp: python type, raw value, units (guessed by
    comparing consecutive deltas and an assumed-seconds diff against host
    wall clock)
  - info().DoF / DoF_m / DoF_e (whether q[0:2] really is the waist)
  - states().q / .dq / .tau lengths and a live sample
"""
import statistics
import sys
import time


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
    print(f"time.time() - float(timestamp) [assumes seconds, same epoch]: "
          f"{now_wall - float(ts)!r}")

    print("\nPolling for 3s to measure timestamp cadence (no sleep, tight loop)...")
    deltas = []
    t_start = time.monotonic()
    last_ts = None
    polls = 0
    while time.monotonic() - t_start < 3.0:
        s = robot.states()
        cur = s.timestamp
        if last_ts is not None and cur != last_ts:
            deltas.append(float(cur) - float(last_ts))
        last_ts = cur
        polls += 1
    print(f"polls={polls}  distinct_timestamp_transitions={len(deltas)}")
    if deltas:
        print(f"delta (raw timestamp units) median={statistics.median(deltas)!r} "
              f"min={min(deltas)!r} max={max(deltas)!r}")
        print("If this system is at 1kHz and these deltas are ~0.001, timestamp "
              "is in seconds (matches offset_estimator.py's assumption).")
        print("If these deltas are ~1.0, timestamp is likely milliseconds.")
        print("If these deltas are ~1000+, timestamp is likely microseconds.")

    print("\nDone. No commands were sent to the robot.")


if __name__ == "__main__":
    main()
