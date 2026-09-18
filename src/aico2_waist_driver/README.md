# aico2_waist_driver

Publishes the waist axes `AGV_Joint1` (yaw) / `AGV_Joint2` (pitch) to
`/joint_states`, read directly off one Rizon controller's 1 kHz RDK stream
via its own RDK session — independent of both arm drivers.

Background and the measurements this is built on: `joint_state_architecture.md`
(sec 1 for the baseline numbers, sec 3 for why the waist gets its own node,
sec 4.2/4.4 for this node's design).

## Why this exists

`RobotStates.q` on either Rizon controller is length 9, not 7: `q[0]` is
waist yaw, `q[1]` is waist pitch, `q[2:9]` is that arm's own 7 joints. Both
arms report the same waist state (`joint_state_architecture.md` sec 1). Today
`flexiv_amr_driver/joint_state_merger` publishes `AGV_Joint1`/`AGV_Joint2` as
a hardcoded `0.0` because nothing reads `q[0:2]` — this node reads it.

It opens a second, read-only RDK session to whichever controller
`source_robot_sn` names (two concurrent RDK sessions to the same controller
are verified to work; see sec 1). It never calls `Enable()`, `Stop()`, or any
command method — it only calls `states()` — so it has no interaction with
whichever driver is actually commanding that arm, and its own lifecycle is
independent of the arm drivers'.

## Running it during the merger transition

`joint_state_merger` also publishes to `/joint_states` with the waist at
0.0. Running both at once means last-writer-wins on `AGV_Joint1/2` between
this node and the merger — expected and useful for verifying this node's
values before the merger is retired (`joint_state_architecture.md` sec 8,
step 3). Watch `ros2 topic echo /joint_states` or RViz's TF for whichever
value is currently winning; if the merger's zeros keep winning, check publish
order/rate rather than assuming this node is broken.

## Parameters

See `config/waist_driver.yaml`. Notable ones:

- `source_robot_sn`: which controller to read from (either arm's serial
  works; defaults to the left arm's).
- `mock_hardware` (default `true`): publishes `0.0` on a timer, no RDK
  connection. Set `false` and provide `source_robot_sn` for real hardware.
- `use_device_timestamp` (default `true`): stamp with the RDK's own hardware
  timestamp (converted via the offset estimator) instead of `now()`.
- `offset_refresh_sec` / `offset_slew_limit`: how often the device-to-host
  clock offset is re-estimated and how far it's allowed to move per refresh
  (a step here can push TF backwards and make tf2 discard its buffer).

## Diagnostics

`~/clock_offset` (`diagnostic_msgs/DiagnosticStatus`) carries the current
offset estimate and its spread — a jump here is the first sign the
controller rebooted.

## Not yet done

Retrofitting the arm drivers to the same poll-thread/device-timestamp
pattern, and retiring `joint_state_merger`, are separate, higher-risk
changes (`joint_state_architecture.md` sec 8, steps 4-6) — left for a
follow-up once this node's output has been checked against real hardware.
