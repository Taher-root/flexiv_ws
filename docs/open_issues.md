# Open issues — 2026-09-26, updated 2026-09-30

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

**Measured 2026-09-30 on Rizon4-063352:** `DoF 9, DoF_m 7, DoF_e 2`.
`AGV_Joint1` (yaw) range -87.45..+87.45 deg, sitting at 0.09 deg.
`AGV_Joint2` (pitch) range **+2.50**..+87.45 deg, sitting at 5.85 deg.

So the merger's hardcoded 0.0 for AGV_Joint2 is **below that axis's
mechanical minimum** -- the rendered model holds the torso in a pose the
robot cannot reach. The yaw error is 0.09 deg, i.e. nothing. Both axes are
genuinely commandable; scripts/rdk_waist_move.py moves them.

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

## 5. RESOLVED (mostly) — the chassis was inhibited, not the driver

**2026-09-30. The control-token theory is disproven.** With
`robokit_velocity_controller` running unchanged — no `pkill`, no swap to
`velocity_controller`, `use_robokit` still defaulting to `true` —
`teleop_twist_keyboard` moved the chassis, and a `/navigate_to_pose` goal then
succeeded in 3.4 s with `number_of_recoveries: 0`.

So a Seer chassis *does* accept `robot_control_motion_req` (2010) from this
driver with no gain-control handshake. The earlier "never acquires control"
reasoning was wrong.

**What was actually happening:** during the failing run the chassis had a
flashing red light — E-stop or chassis fault. It accepted every `/cmd_vel`
packet and ignored it. Signature at the time: `/cmd_vel` non-zero
(0.189 m/s), robot creeping 7 cm in 108 s, `number_of_recoveries: 12` from
`SimpleProgressChecker` (0.5 m required within 10 s → ~9 s per cycle → 12
cycles in 108 s). Once the light was cleared, everything worked with no
config change. Not proven retroactively, but it fits every observation.

**Also ruled out** (both checked *after* the successful run, config
unchanged throughout): `collision_monitor` — `scan.enabled` read `True` and
`PolygonStop.radius` read `0.3` across both the failure and the success;
`/scan/merged` was healthy at ~32 Hz with `/scan/nav` and `/scan/avoid` both
at 15 Hz.

**Still worth fixing in `robokit_velocity_controller.py`** — independent of
the above, two real defects make any chassis-side refusal invisible:

- The reply is read and discarded: `data = self.sock.recv(16)` inside
  `except socket.timeout: pass`, and `data` is never inspected. An explicit
  error code on every 20 Hz packet would go unseen.
- `recv(16)` reads exactly the header (`!BBHLH6s`) and leaves the JSON body
  in the socket buffer, so the next `recv(16)` parses body bytes as a header.
  The stream desynchronises after the first reply.

Parsing and logging the reply is ~15 lines and needs no API spec.

---

## 6. Chassis E-stop and fault state are invisible to the stack

**Confirmed 2026-09-30.** `ros2 topic echo /amr/emergency --once` hung with no
output, so `/amr/emergency` is advertised but never published:
`status_monitor.connect_to_amr()` failed (either `import flexivamr` or
`states_api.connect()`), leaving `self.states_api = None`, after which
`publish_status()` returns early every tick. The node logs the error once at
startup and is silent thereafter.

Two consequences, and together they cost about an hour of debugging:

1. The only node that reads `check_emergency_status()` /
   `check_blocked_status()` was producing nothing, so an E-stop was
   unobservable from ROS.
2. Even when it works, **nothing consumes** `/amr/status`,
   `/amr/emergency` or `/amr/blocked`. Nav2 plans, commands, and burns
   recoveries against an inhibited chassis with no indication why.

**Fix, in order of value:**

- Make `status_monitor` retry the connection instead of giving up at startup,
  and log a throttled warning while `states_api is None` so the failure is
  visible in a running system.
- Surface it where it is needed: gate goal acceptance, or at minimum log
  loudly, when `emergency_stop` is true. A `/navigate_to_pose` goal against an
  E-stopped chassis should say so rather than time out via the progress
  checker.

**Check first:** `ros2 topic hz /amr/status` and
`python3 -c "import flexivamr"` on the Jetson. If the import fails, the
library is missing from that machine and `docs/porting.md` needs it.

---

## 7. Smaller items

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
