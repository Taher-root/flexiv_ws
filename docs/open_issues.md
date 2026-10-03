# Open issues — 2026-09-26, updated 2026-10-03

Parked for tomorrow. Each entry says what was measured, what the code says,
and what is still a hypothesis. Nothing here is blocking a port.

---

## 1. RESOLVED — MoveIt execution was jerky: TOTG left jerk unbounded

**Root cause, measured 2026-10-02/03.** `AddTimeOptimalParameterization`
(TOTG) is *time-optimal*, which means it saturates acceleration. The result
is a trapezoidal velocity profile — accelerate, cruise, decelerate — whose
acceleration steps discontinuously at the two corners. Jerk is unbounded
exactly there, and on hardware a single-joint move read as **three distinct
segments**.

**Fixed** by adding `AddRuckigTrajectorySmoothing` after TOTG in
`ompl_planning.yaml`, plus the jerk limits it requires in
`joint_limits.yaml` (there were none for any joint). Ruckig re-times the same
path under a jerk limit, replacing each corner with a bounded acceleration
ramp.

### The experiment that identified it

The same 30-degree joint-4 move at `trajectory_send_rate_hz` **2.0 and 50.0**
produced **identical** behaviour — three segments both times. A 25x change in
delivery rate changing nothing ruled out the send loop, the controller's
internal re-planning, and GIL contention in one comparison, and located the
problem in the trajectory itself.

### Three wrong mechanisms were recorded here before that

Worth keeping, because each was argued from code and each was wrong:

1. *"`default_max_joint_vel` is a fixed 1.5 rad/s, so the controller sprints
   and idles 50 times a second."* Arithmetically impossible: at
   `max_acc` 3.0 rad/s^2, 2 ms of acceleration covers 6e-6 rad, not the
   ~0.012 rad of a 20 ms segment.
2. *"`SendJointPosition` re-plans on every call, so the internal generator
   never completes a segment."* Plausible, and killed by the 2 Hz vs 50 Hz
   result above.
3. *"Delivery jitter from GIL starvation produces discrete lurches."* The
   send loop turned out to be uniformly slow (36.7, 39.6, 37.9 Hz), not
   bursty — a real bug (see issue 2a) but not this one.

The lesson is in the method: all three were consistent with code that could
be read, and none required the hardware. The one-parameter sweep settled it.

### Jerk limit values

`max_jerk` is set to roughly 10x each joint's acceleration limit — 10.0 for
joints 1-2, 12.0 for 3-4, 24.0 for 5-7, 5.0 for the waist axes. The Rizon4
datasheet does not publish jerk limits, so these were chosen for feel. Lower
is smoother and slower; that is the knob.

### Still worth doing (not the cause, but wrong)

`_max_vel` / `_max_acc` handed to `SendJointPosition` are still the constants
1.5 and 3.0, unrelated to the trajectory. MoveIt's effective limits after
`0.3` scaling are 0.63-1.47 rad/s and 0.3-0.72 rad/s^2, so the caps are
generous rather than binding — but deriving them from
`trajectory.points[*].velocities` is ~10 lines and removes a mismatch that is
wrong on its own terms.

---

## 1a. The call-rate sweet spot is ~50 Hz, and it is a U-curve

**Measured after the Ruckig fix and the acquisition fix, so rates are exact.**

| `trajectory_send_rate_hz` | achieved | result |
| --- | --- | --- |
| 2 | 2 Hz | badly jerky |
| 50 | 50 Hz | smoothest |
| 100 | 98 Hz | slightly jerky |
| 200 | 190 Hz | clearly jerky |

Not a slope — a minimum around 50 Hz. Two readings, both tentative:

- **2 Hz being bad says the internal generator does not blend.** The driver
  passes a target velocity in `dq_send`. If the generator honoured it and
  re-planned from the arm's measured velocity, widely spaced commands would
  be smooth. They are not, so it appears to decelerate toward each commanded
  position instead.
- **Degradation above 100 Hz says the generator is being pre-empted** before
  it can produce a smooth segment.

That matches the RDK documentation describing NRT joint modes as expecting
commands "in a one-shot or slow-periodic manner" — except that 2 Hz, which is
what "slow-periodic" sounds like, is the worst setting measured. The docs give
a number for RT (1 kHz) and none for NRT.

**Not yet swept:** 20 and 30 Hz. The minimum may sit below 50.

**Asked of Flexiv support (2026-10-03):** whether there is a recommended call
rate, whether a complete externally time-parameterised joint trajectory can be
handed over as one motion, and whether RT control is available for the AICO2-4
at all. For the record: `flexivrdk` 1.9.0 exposes no RT modes and no `Stream*`
methods — verified on the robot — and 1.9.0 has no multi-waypoint function
(`dir(Robot)` gives only `ExecutePlan`, `PausePlan`, `StopPlan`).

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
the Jetson terminal after a move to find out.

Issue 1 is resolved and was not the cause, so this stands on its own. Worth
re-measuring now that trajectories are jerk-limited: a smoother approach may
land inside tolerance where a trapezoid's abrupt stop did not. Observed
residuals on a successful move are 0.05-0.54 degrees against a 1.15 degree
tolerance, so convergence is not currently failing.

---

## 2a. RESOLVED — acquisition busy-loop starved the send loop, and segfaulted on teardown

**Measured and fixed 2026-10-02.** Two bugs in the same thread, neither
causing issue 1, both real.

### The busy-loop

`_poll_loop` called `states()` with no sleep. `states()` is a ~1.5 us cached
read, so the thread ran at **~615,000 polls/s** against a device that
produces ~1000 samples/s — a full CPU core per arm, two cores on a Jetson
also running Nav2, RTAB-Map and the camera pipelines.

It also stole the GIL. A CPU-bound Python thread holds it for up to
`sys.getswitchinterval()` at a time, and the trajectory send loop has to
reacquire it after every `time.sleep()`. The driver's own instrumentation
caught the result:

```
trajectory sent 152 setpoints in 4.01s = 38 Hz (configured 50 Hz)
  <- loop is saturated; the configured rate is not being reached
```

Per-second: 36.7, 39.6, 37.9 Hz. Uniformly 24% short, not bursty.

**Fixed** with `time.sleep(0.0005)` per iteration — still 2 kHz, twice the
device rate, so no sample can be missed. After:

| configured | achieved |
| --- | --- |
| 2 Hz | 2 Hz |
| 50 Hz | 50 Hz |
| 100 Hz | 98 Hz |
| 200 Hz | 190 Hz |

Exact, and the saturation warning is gone. That also identified what the
~6.3 ms per cycle had been: **GIL wait, not the RDK round trip.**
`SendJointPosition` is cheap.

### The teardown crash

A redundant `ACTIVATE` raises `RCLError("Transition is not registered")` from
inside rclpy's own lifecycle service callback, which propagates out of
`executor.spin()`. That is not a `KeyboardInterrupt`, so it fell through to
the bare `finally`, which calls `destroy_node()` — and `destroy_node()` does
not run lifecycle callbacks, so the poll thread kept calling `states()` while
the `flexiv::rdk::Robot` it holds was freed. The process died with **SIGSEGV
(exit code -11)** behind a "Missed 1 heartbeat signal transmission" warning.

This presented later as an unexplained `NOT_ENABLED` — the driver had
crashed, the RDK session died without a clean release, and the arm was left
with its servo off. It looked like an E-stop and was not.

**Fixed** by extracting the poll-thread stop into an idempotent
`stop_acquisition()`, calling it from the `finally` before `destroy_node()`,
and catching `Exception` so an unexpected error is logged with its cause.

**Still unexplained:** where the duplicate `ACTIVATE` came from. The
autostart handler in `arms.launch.py` pins activate to `start_state:
"configuring"` precisely to avoid this, so a second launch process is the
most likely source — two were alive at the time. The crash is now
non-fatal either way.

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
