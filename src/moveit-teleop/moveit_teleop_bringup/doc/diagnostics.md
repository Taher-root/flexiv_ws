# Diagnostics

What to look at, in the order that resolves things fastest.

| Symptom | Look at | Usual cause |
|---|---|---|
| Everything is up and the arm does not move | `/mode_manager/status` | Mode is not `TELEOP`. Check `input_healthy` and `input_age`. After any HALT the clutch needs a **fresh release** before it can re-engage — deliberate, so an operator still holding the clutch through a dropout does not have the arm resume under their hand. |
| Arm hits something the wall should have stopped | `/feedback/tcp_pose` vs FK of the command | `wall_clearance` sized against geometry rather than tracking error, and/or the loop closed on the command echo. See [tuning step 5](tuning.md#5-size-wall_clearance-against-tracking-error). |
| Sluggish | `/ik_node/status` → `rate_limited`, `solve_time_p99` | If `rate_limited` is rarely true and p99 is well inside the frame period, the lag is the driver and arm, and no config value reaches it. |
| Arm freezes near a surface | `ik_node` log → `refusing a N deg jump` | Branch flips refused with no nearby solution. Check `jump_retry_smoothness` is not 0 — the startup guard line reports `retry x<value>`. |
| Wrist wanders on straight moves | joint5 near zero, `corr(dj4,dj6)` | Wrist singularity. [Tuning step 4](tuning.md#4-fix-the-wrist-with-weight_orientation). |
| Arm re-anchors constantly | `retarget_node` log → `anchored` | `external_motion_threshold` too close to `max_target_lead`, or a wall switching on and off under the target. |
| Recorder refuses every episode | `episode_recorder` log | A required stream is not being published. Almost always a `${...}` topic that does not exist for this solver/device combination. |
| Panel lists no episodes | `episodes_dir` in the launch log | It is derived from `recording.output_dir`. If you passed `record_dir`, the panel still points at the config's directory. |
| Launch aborts naming a config key | the error text | The key does not exist. It lists the known ones; check [config-reference.md](config-reference.md) rather than assuming a rename. |
| Homing times out on hardware | `/feedback/arm_status` | CAN gate closed, or `ctrl_mode: 2` — the arm is in drag-teach mode and ignoring CAN commands **by design**. |
| Headset cannot load the page | `xr_device` startup log | Host firewall. The node prints the exact `ufw` rule, and warns only when ufw is genuinely filtering rather than merely installed. |
| `address already in use` on 8443 | — | A leftover `xr_device` from a previous run: `pkill -f xr_device`. |
| `homing by direct interpolation (...)` at WARN | `mode_manager` | `move_group` could not plan the path, so the arm is homing on an **unchecked** straight interpolation. The named code says why; `START_STATE_IN_COLLISION` usually means the arm is parked touching modelled geometry. |
| `homing refused: CONTROL_FAILED` / `GOAL_IN_COLLISION` | `mode_manager` | Deliberately does **not** fall back: either something already executed, or the destination posture itself collides — check `ready_state` against the scene. |
| `homing waits for the planning scene` | `mode_manager` | `scene_objects` has not reported the scene applied. Planning before it would be collision-aware against an empty world. Bounded by `scene_ready_timeout`. |

---

## The four startup lines worth reading every time

```
1. group='arm' chain 'base_link' -> 'tcp_link' (6 joints) controller='arm_controller'
2. nominal posture from ... condition number N        <- is the bias posture sane
3. speed from ... x rate_limit_scale -> [...] rad/s   <- what ceiling is in force
4. guards: joint limits ..., jump limit N deg (retry xN),
           collision policy 'stop', M virtual wall(s) <- anything OFF prints at WARN
```

Read line 4 before trusting any tuning measurement: a guard that reports off
(`retry x0`) while it is in fact firing invalidates the comparison.

---

## Topics worth having open

| Topic | Carries |
|---|---|
| `/mode_manager/status` | mode, recording flag, `input_healthy`, `input_age` |
| `/ik_node/status` | solve time and p99, position/orientation error, `limited_joints`, `rate_limited`, `collision_blocked`, `refused_jumps` |
| `/teleop/feedback` | wall contact: which walls, how deep |
| `/ik_node/pose_target_cmds` | the commanded pose, after retargeting |
| `/feedback/tcp_pose` | where the arm *actually* is (Piper driver) |

```bash
ros2 topic echo /mode_manager/status --field mode
ros2 topic hz /ik_node/pose_target_cmds
```

---

## Analysing a recorded episode

Episodes are MCAP bags plus a JSON sidecar. The sidecar carries what a bag
cannot: the task label, whether the episode ended cleanly or faulted out, and a
timeline of mode transitions.

```bash
python3 -c "
import json; d=json.load(open('EPISODE/episode.json'))
print(d['end_reason'], d['halted_during_episode'])
for m in d['mode_timeline']: print(m)
"
```

Two comparisons answer most questions:

- **Commanded vs real.** FK of `/control/joint_states` against
  `/feedback/tcp_pose`. Their difference is the tracking error, and its
  best-fit time shift is the phase lag. On the Piper: 46 mm median, ~140 ms.
- **Sustained joint rate vs the cap.** Differentiate joint positions over
  50–500 ms windows, not single samples — instantaneous overspeed is
  `JointTrajectoryController` spline ripple between setpoints, while sustained
  displacement is real.

> **Beware of `/control/joint_states` on Piper hardware.** It is the *command*
> bus: the driver **subscribes** to it. Real state is `/feedback/joint_states`
> and `/feedback/tcp_pose`. Its `velocity` field is also published as all
> zeros, so velocity must be differentiated from position.
