# Open issues — 2026-09-26

Parked for tomorrow. Each entry says what was measured, what the code says,
and what is still a hypothesis. Nothing here is blocking a port.

---

## 1. MoveIt execution is jerky — velocity cap far above the trajectory

**Confirmed from code, not yet from a measurement.**

`arm_driver_node._execute_trajectory` sends every setpoint with a fixed cap:

```python
self._session.send_joint_position(q_send, dq_send, self._max_vel, self._max_acc)
```

`_max_vel` is `default_max_joint_vel` = **1.5 rad/s on every axis**, unrelated
to the trajectory being run. `SendJointPosition` re-plans inside the controller
on every call, so at 50 Hz the controller sprints at 1.5 rad/s toward a
setpoint a fraction of a degree away, arrives in ~2 ms, idles ~18 ms, and is
aborted by the next send. Accelerate–arrive–wait, 50 times a second.

The driver's own parameter docstring already describes this failure mode.
At RViz `Velocity Scaling: 0.30` against URDF limits of 2.09–4.89 rad/s, the
trajectory's real peak is well under 0.6 rad/s for a normal move.

**Workaround (live, no relaunch):**
```bash
ros2 param set /left_arm/left_arm_driver default_max_joint_vel 0.6
ros2 param set /left_arm/left_arm_driver default_max_joint_acc 1.2
```
Too low and the arm lags the trajectory and starts failing goal tolerance.

**Real fix (~10 lines, not written):** derive the cap per trajectory from the
message. MoveIt parameterises with TOTG, so peak `|dq|` per joint is already in
`trajectory.points[*].velocities`. Use `max|dq| * margin` instead of a constant
and the tuning disappears.

---

## 2. Long dead time after a move completes

**Confirmed from code.** After the trajectory's nominal duration the driver
keeps polling until the worst joint is inside `goal_joint_tolerance`
(0.02 rad = 1.15°), or until:

```python
settle = min(self._goal_settle_timeout, 0.5 * duration + 0.5)   # cap 3.0 s
```

If it never converges it burns the full settle — up to **3 s with the arm
visibly stopped** — then aborts with `GOAL_TOLERANCE_VIOLATED`, which RViz
reports as a generic failure.

**Unknown:** whether it is converging. The driver logs
`goal tolerance not reached: <joint> is X.XX° from target` on failure — read
the Jetson terminal after a move to find out. Issue 1 may be causing the
residual, in which case fixing the cap fixes both.

---

## 3. Waist joints are hardcoded to 0.0

`joint_state_merger` initialises `AGV_Joint1`/`AGV_Joint2` to `0.0` and never
updates them — nothing publishes the real values. The torso renders at the
wrong yaw/pitch, and both arms hang off `AGV_Pitch`, so the whole upper body is
drawn rotated away from reality.

`enable_waist_driver:=true` is **not** the fix: `waist_driver` opens a *second*
RDK session to the same controller and, as of 2026-09-26, was never observed
reaching `active`. Suspected connect failure; unverified.

**Real fix:** `arm_driver_node._extract_arm_q` already reads the full 9-DoF `q`
off the left arm's existing session and discards indices 0–1, which are exactly
the waist. Publishing them needs no second session, no extra node and no flag.
Proposed as a `publish_waist_joints` parameter, off by default.

**Constraint to respect:** `robot_state_publisher` does **not** merge partial
JointState messages — see `joint_state_architecture.md`. Exactly one publisher
must emit all 16 joints in one message, so this change must keep that property.

---

## 4. `direct_joint_states:=true` is unusable

Follows from the RSP finding above. Arms publish 7-joint messages, RSP
publishes transforms only for the joints in each message, waist transforms
never appear, the upper body detaches. Leave it `false` until issue 3 lands.
The flag and its docs are corrected in `joint_state_architecture.md`.

---

## 5. Chassis does not move on `/cmd_vel` (intermittent)

Nav2 emits correct velocities — `/cmd_vel` and `/cmd_vel_smoothed` were
observed identical and non-zero, so the collision monitor is passing them
through. `/odom_raw` twist read zero at the same time (single sample, not
conclusive).

`robokit_velocity_controller` **never acquires control**: no gain-control
message exists anywhere in `robokit_protocol.py`, which defines only 1004,
1005 and 2010. `velocity_controller` (the non-Robokit one) does call
`configure_api.gain_control('ros2_nav')`. A Seer chassis refuses motion from a
client without the control token, and this driver cannot tell — it does
`self.sock.recv(16)`, discards the header and swallows timeouts, so a refusal
is indistinguishable from success.

`use_robokit` is now a real argument on `full_system.launch.py` (it was
hardcoded `true`), but **that fix has not been transferred to the Jetson**, so
`use_robokit:=false` there is still silently ignored.

**Decisive test, not yet run:** stop the full stack, then
`amr_driver.launch.py use_robokit:=true` alone, `ros2 topic pub -r 10 /cmd_vel`
a slow spin, and watch `/odom_raw`. Repeat with `use_robokit:=false`.

**Blocked on:** the Seer API spec for the gain-control request ID and body.
Without it, adding acquisition to the Robokit driver is guesswork.

---

## 6. Smaller items

- `/amr/actual_velocity` has **two publishers**: `odometry_publisher` (real
  encoder speed, API 1005) and `robokit_velocity_controller` (echoes the
  *command* back as if measured). Last writer wins. Do not use it to confirm
  motion — use `/odom_raw`.
- `both_arms` has a planner config in `ompl_planning.yaml` but no solver in
  `kinematics.yaml`, so selecting it gives no interactive marker and 14-DoF
  planning that took ~123 s. Either add a note or remove the config.
- `longest_valid_segment_fraction: 0.005` is unusually tight; 0.01–0.02 would
  plan 2–4× faster. Not changed — collision-checking resolution needs a
  deliberate decision.
- Four visual meshes (`AGV_Base.obj`, `AGV_Pitch.obj`, `link2.obj`,
  `link6.obj`) are still absent from the repo; the URDF points those five
  references at the collision STLs instead. Real `.obj` files exist on both
  machines under `~/flexiv_ws/src/flexiv_amr_description/meshes/visual/`.
  `AGV_Base` and `AGV_Pitch` are 27–30 MB each, which is why committing them
  was deferred.
- Six `.mtl` files (`link0`, `link3`, `link3r`, `link4`, `link7`, `ring`) are
  also absent, so those links render without materials.
- Never run: RTAB-Map mapping, `use_device_timestamp:=true` on the left arm.
- Unexplained, from 2026-09-25: a ~150° jump on joints 1 and 7 between two
  read-only commands.
