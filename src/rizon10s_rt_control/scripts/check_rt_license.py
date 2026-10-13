#!/usr/bin/env python3
"""Does this arm carry an RDK licence that permits the RT control modes?

Run this before asking Flexiv for anything. The answer is already on the robot:
RobotInfo carries a license_type field (rdk v1.9 include/flexiv/rdk/data.hpp),
and SwitchMode's contract is "throw std::invalid_argument if the requested mode
is invalid or unlicensed" (robot.hpp). So the licence, not the API, is the gate.

This only reads. It never switches mode and never enables the robot, so it is
safe to run with the arm parked -- though it does take the one RDK session, so
stop the Python driver first.

    python3 check_rt_license.py Rizon4-063352
    python3 check_rt_license.py Rizon4R-062077
"""
import sys

try:
    import flexivrdk
except ImportError:
    sys.exit("flexivrdk not importable -- run this on the Jetson, in the same "
             "environment the arm driver uses")

FIELDS = ("serial_num", "software_ver", "model_name", "license_type",
          "DoF", "DoF_m", "DoF_e")


def main() -> int:
    if len(sys.argv) < 2:
        print(f"usage: {sys.argv[0]} <robot_sn>", file=sys.stderr)
        return 1
    robot = flexivrdk.Robot(sys.argv[1])
    info = robot.info()

    print("--- RobotInfo ---")
    for name in FIELDS:
        value = getattr(info, name, None)
        if value is None:
            print(f"{name:14s} <not exposed by this wheel>")
        else:
            print(f"{name:14s} {value}")

    # The Python bindings in v1.9 expose the NRT modes only: upstream names
    # every example_py file non_realtime_* and every RT example is C++. So an
    # empty list here is the expected, current state of the wheel and says
    # nothing about the robot's licence -- that is what license_type is for.
    modes = [m for m in dir(flexivrdk.Mode) if not m.startswith("_")]
    print("\n--- Mode enum, as the Python wheel exposes it ---")
    print("RT modes:  ", sorted(m for m in modes if m.startswith("RT_")))
    print("NRT modes: ", sorted(m for m in modes if m.startswith("NRT_")))
    print("\nStream* methods:",
          sorted(m for m in dir(robot) if m.startswith("Stream")))

    print("\nRT control needs all three: a licence that permits the RT modes\n"
          "(license_type above), the C++ library (the wheel has no Stream*),\n"
          "and a host that can hold the 1 kHz loop (rt_hold_probe).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
