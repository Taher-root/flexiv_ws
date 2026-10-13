# aico2_msgs

Shared interfaces for the AICO2 stack (see `docs/AICO2_ROS2_Plan.md` §3.3).

## Messages

- `SystemState` — supervisor mode (IDLE / MANIPULATING / NAVIGATING / …)
- `ChassisStatus` — FMR blocked, emergency, battery, station
- `ArmStatus` — fault, operational, mode

## Services

- `ExecutePrimitive` — Flexiv RDK primitive string
- `Relocate` — chassis relocalization
- `GainControl` — seize/release FMR control
- `RequestMode` — ask safety supervisor for manip vs nav

## Actions

- `NavToStation` — `fixed_path_navigation` wrapper (Path A)
