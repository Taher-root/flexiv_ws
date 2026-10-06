# Open issues — 2026-09-26, updated 2026-10-03

Parked for tomorrow. Each entry says what was measured, what the code says,
and what is still a hypothesis. Nothing here is blocking a port.

---

## 1. RESOLVED — execution was jerky: TOTG left jerk unbounded

`AddTimeOptimalParameterization` is time-optimal, so it saturates
acceleration: the velocity profile is a trapezoid whose acceleration steps
discontinuously at the two corners. Jerk is unbounded there, and a
single-joint move read as three distinct segments.

**Fixed** by adding `AddRuckigTrajectorySmoothing` after TOTG in
`ompl_planning.yaml`, plus the jerk limits it requires in `joint_limits.yaml`
(there were none). `max_jerk` is ~10x each joint's acceleration limit, chosen
for feel — the Rizon4 datasheet publishes no jerk limits. Lower is smoother
and slower.

**Identified by** running the same move at `trajectory_send_rate_hz` 2.0 and
50.0: identical both times, which ruled out the send loop, the controller's
re-planning and GIL contention at once. Three earlier mechanisms recorded
here — the velocity cap, 50 Hz re-planning, and delivery jitter — were all
argued from code without the hardware, and all wrong.

**Do not "fix" the caps.** `_max_vel` / `_max_acc` passed to
`SendJointPosition` are the constants 1.5 and 3.0 on every axis, unrelated
to the trajectory. That looks wrong, and deriving them per trajectory from
its own peak `|dq|` / `|ddq|` was implemented and **reverted**: it made the
motion jerky again at the same 50 Hz that was smooth with the constants.

Why, measured: at `velocity_scaling` 0.3 the derived acceleration cap is
1.44 rad/s^2 against the constant 3.0, and at 0.05 scaling it is 0.24 — a
12x reduction. The controller plans its own motion to each commanded
position (see issue 1a: it decelerates toward each setpoint rather than
blending), so it needs acceleration headroom well above what the trajectory
itself demands. A cap matched to the trajectory starves it.

The constants are not a mismatch to be tidied up. They are headroom.

---

## 1a. Call rate is a U-curve with a minimum around 50 Hz

Measured after the fixes above, so achieved rates are exact:

| configured | achieved | result |
| --- | --- | --- |
| 2 | 2 Hz | badly jerky |
| 50 | 50 Hz | smoothest |
| 100 | 98 Hz | slightly jerky |
| 200 | 190 Hz | clearly jerky |

**Explained by Flexiv (2026-10-05).** The two readings recorded here earlier
were each partly wrong. The mechanism, from their engineering team:

`SendJointPosition` does not create a trajectory segment. **Each call replaces
the current target** (position and velocity), and a motion generator on the
robot, running at 1 kHz, chases the most recent target as fast as `max_vel` and
`max_acc` allow. It has no timing information — it does not know when the next
command is due, so it is never synchronised to the send rate — and **no jerk
limit**, so acceleration switches abruptly between `+max_acc`, 0 and `-max_acc`.

- **2 Hz.** The generator does try to pass through each sample at the velocity
  sent, so that reading was close. What it cannot do is hold a velocity at a
  fixed point: when the limits exceed what the trajectory needs it **arrives
  early**, overshoots, brakes at `max_acc`, and settles back, because the target
  does not move until the next command. That is the stop-and-go.
- **Above 100 Hz.** Not pre-emption — that reading was wrong. The generator
  re-plans every millisecond regardless, so nothing is pre-empted mid-segment.
  It is **timing jitter**: at 5–10 ms periods, jitter from Python and the
  non-real-time network is a large fraction of the period, targets arrive
  unevenly, and each uneven step provokes a correction at up to full `max_acc`.


**Not swept:** 20 and 30 Hz. The minimum may sit below 50.

**Answered by Flexiv support (2026-10-05):** RT control **is** available on
the Rizon series with the RDK-Professional licence we hold, but only from
**C++** — the Python bindings are NRT-only by design, which matches what we
measured. RDK v1.x up to v1.9.3 is the Rizon line; v2.x is Enlight only, so
the 2.x `ProductModel` enum reading was right.

`RT_JOINT_POSITION` + `StreamJointPosition(pos, vel, acc)` takes no
`max_vel` / `max_acc` and does no internal planning, so this whole U-curve is
structurally absent in RT: there is no setpoint-to-setpoint planning to
pre-empt or starve. It is the real fix rather than a tuning exercise.

Not pursued yet, and NRT stays the default: RT means owning a 1 kHz loop on a
Jetson with no PREEMPT_RT kernel that also runs Nav2, RTAB-Map and two camera
pipelines, and a missed deadline in RT is worse than NRT because nothing
interpolates for you. Evaluation scaffolding is in `aico2_rt_control`, which
builds nothing until the C++ RDK is installed and changes nothing in the
existing driver. Start with its `rt_hold_probe`.

### Both remaining questions are now answered

**Recommended NRT rate: there isn't one, by design.** Flexiv do not publish a
figure because the mode "is designed for one-shot or slow-periodic targets, not
for streaming a trajectory". Our 50 Hz fits how the generator works, but the
optimum depends on `max_vel`, `max_acc` and the host's jitter. Their tuning
advice, if staying on NRT: steady rate around 20–50 Hz with minimal jitter,
velocities consistent with the surrounding position samples, and `max_vel` /
`max_acc` set *just above* the trajectory's real peaks rather than large.
Their own conclusion: "even when tuned, NRT mode will not reproduce your
time-parameterization exactly."

**Note the conflict with what we measured**, because it matters if anyone
revisits this. Deriving the caps from each trajectory's peaks is exactly the
advice above, and it made the motion jerkier — reverted in `f850e16`, with
`trajectory_cap_margin 100.0` (effectively the old large constants) confirming
the cause. The likely reconciliation is the word *just*: a generator chasing a
target that is already up to 20 ms stale has to catch up, and catching up needs
acceleration above the trajectory's own peak. Capped at the peak it falls
behind, then corrects hard. A margin between the two — say 1.5–3× rather than
1× or 100× — was never swept. Nobody should spend time on that sweep now: the
decision is RT, where there are no caps at all.

**Uploading a time-parameterised trajectory: not possible.** There is no API to
hand the robot a time-stamped joint trajectory for it to replay with our
timing. The alternatives, per Flexiv, both give up the timing:

- `MoveJ` with multiple waypoints and `zoneRadius` blending, via
  `ExecutePrimitive()`.
- `MoveJTraj`, which replays a `.traj` file uploaded with
  `FileIO::UploadTrajFile()`. A `.traj` holds per-segment waypoints with
  velocity, acceleration and blending — **no timestamps**.

Use those only where the path matters more than the timing. For our timing, the
answer is RT, which is what Flexiv also recommend.

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

## 2a. RESOLVED — acquisition busy-loop, and a segfault on teardown

Two bugs in the same thread, found while investigating issue 1 and not its
cause.

**Busy-loop.** `_poll_loop` called `states()` with no sleep. It is a ~1.5 us
cached read, so the thread ran at ~615,000 polls/s against a device producing
~1000 samples/s — a core per arm — and stole the GIL from the trajectory send
loop, which achieved 38 Hz against 50 configured and logged its own
saturation warning. Fixed with `time.sleep(0.0005)` (still 2 kHz, twice the
device rate). Rates are now exact: 2/2, 50/50, 98/100, 190/200, warning gone.
That also showed the ~6.3 ms per cycle was GIL wait, not the RDK round trip.

**Teardown segfault.** A redundant `ACTIVATE` raises
`RCLError("Transition is not registered")` out of rclpy's lifecycle callback
and through `executor.spin()`. Not a `KeyboardInterrupt`, so it reached the
bare `finally`, which calls `destroy_node()` — which does not run lifecycle
callbacks, so the poll thread kept calling `states()` on a freed
`flexiv::rdk::Robot`. SIGSEGV, exit code -11. This later presented as an
unexplained `NOT_ENABLED` that looked like an E-stop. Fixed with an
idempotent `stop_acquisition()` called from the `finally` before
`destroy_node()`, plus catching `Exception` so the cause is logged.

**Still unexplained:** where the duplicate `ACTIVATE` came from. The
autostart handler pins activate to `start_state: "configuring"` to prevent
exactly this, so a second launch process is the likely source — two were
alive at the time. Non-fatal either way now.

---

## 3. RESOLVED — waist was hardcoded to 0.0 in the merger

**Measured 2026-09-30 on Rizon4-063352:** `DoF 9, DoF_m 7, DoF_e 2`.
`AGV_Joint1` (yaw) range -87.45..+87.45 deg. `AGV_Joint2` (pitch) range
**+2.50**..+87.45 deg — so the merger's 0.0 was *below that axis's mechanical
minimum*, and every TF consumer was handed a pose the robot cannot reach.

The merger was worse than recorded here. `LEFT_SET` held only `Left_joint*`,
so with `control_waist:=true` the left driver's `AGV_Joint1/2` did not merely
fail to merge — they hit the unknown-name branch and logged an error per
joint about the wrong driver publishing.

**Fixed** by accepting the waist on the left topic
(`LEFT_SET = set(LEFT) | set(WAIST)`), which is the correct owner: the AICO2
manual states the right arm's waist data is transmitted via the left arm's
communication module. The seed is now the measured mechanical minimum
(`AGV_Joint2 = 0.0436 rad = 2.50 deg`) rather than 0.0, so the pose is
reachable before the first real reading arrives.

Needs `control_waist:=true` for real values. `enable_waist_driver:=true` is
still **not** the route — `waist_driver` opens a *second* RDK session to the
same controller and was never observed reaching `active`.

## 4. RESOLVED — `direct_joint_states` removed

The flag remapped the drivers straight to `/joint_states` and skipped the
merger, on the belief that `robot_state_publisher` merges partial
`JointState` messages by joint name. It does not: `robot_state_publisher.cpp`
declares its joint map as a local rebuilt per message, so a partial update
blanks every joint it omits. Arms publish 7-joint messages, the waist
transforms never appear, and the upper body detaches.

It could only ever break TF, so it is **deleted** rather than documented —
all nine references across the three launch files, plus the stale prose in
`flexiv_amr_bringup/README.md`. The merger is now unconditional.

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

- **Fixed:** `/amr/actual_velocity` had *three* publishers, not two —
  `odometry_publisher` (real encoder speed, API 1005) plus **both** velocity
  controllers echoing the *command* back as if measured. The two fake ones
  are removed; only the measurement remains.
- **Documented, cannot be fixed with KDL:** `both_arms` has an
  `ompl_planning.yaml` config and no `kinematics.yaml` entry. It is a
  composite of two subgroups, so it has two tip links and no single
  base->tip chain, and `KDLKinematicsPlugin` requires a serial chain.
  Joint-space planning works without IK (the ~123 s 14-DoF plan); pose goals
  and the RViz marker do not. A dual-arm Cartesian goal needs a multi-tip
  solver such as bio_ik. Recorded in `kinematics.yaml`.
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
