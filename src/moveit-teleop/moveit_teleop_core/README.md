# moveit_teleop_core

The robot-side half of the teleoperation stack: turn normalized device input
into an end-effector pose target, decide when anything is allowed to move, and
publish the geometry the arm must not drive into.

Nothing here names a link, a joint, a frame, a controller or a vendor. Every
robot-specific answer is fetched at runtime from the running `move_group`
(`robot_info.py`) or from that robot's `teleop.yaml`.

```
device input  ->  retarget_node  ->  pose target  ->  a solver (moveit_teleop_ik)
                      ^
                      |  mode_manager decides whether that is allowed at all
```

Architecture and signal flow:
[moveit_teleop_bringup/doc/architecture.md](../moveit_teleop_bringup/doc/architecture.md).

---

## Nodes

### `retarget_node` — device pose → end-effector target

Clutched relative-pose retargeting: while the clutch is held, the tool follows
the *change* in the device pose since the clutch went down, scaled, smoothed,
projected off virtual walls and leashed to the tool's actual position.

| | |
|---|---|
| Subscribes | `input_topic` (`TeleopInput`), `status_topic` (`TeleopStatus`), the measured joint states |
| Publishes | `~/target_pose` (`geometry_msgs/PoseStamped`) — remapped by the bringup to the chosen solver; `feedback_topic` (`TeleopFeedback`) |
| Rate | one output per input sample, not a timer |

| Parameter | Default | |
|---|---|---|
| `move_group` | `arm` | the SRDF group; base link, tip link and joints come from it |
| `ee_link`, `planning_frame` | `''` | override what the SRDF says; normally leave empty |
| `description_source` | `/move_group` | whose parameters the robot model is read from |
| `retargeter` | `relative_pose` | strategy, loaded from the `moveit_teleop.retargeters` entry-point group |
| `translation_scale` | `0.5` | device metres → robot metres |
| `rotation_scale` | `1.0` | 1.0 is twitchy on a wrist-held controller |
| `translation_ref` / `rotation_ref` | `base` / `tool` | `base`, `device` (operator heading at clutch engage) or `tool` |
| `device_convention` | `ros` | `webxr` converts x-right/y-up/−z-forward |
| `smoothing_alpha` | `1.0` | EMA weight per new sample; 1.0 disables |
| `max_translation_step` | `0.05` | refuse a device teleport (applied to the **scaled** delta) |
| `max_target_lead` / `max_target_lead_angle` | `0.05` / `0.5` | how far the target may lead the tool |
| `max_command_step` | `0.03` | how far a **wall** may move the command beyond what the hand drove, before re-anchoring |
| `external_motion_threshold` | `0.15` | tool found somewhere it was not commanded → re-anchor. Must exceed `max_target_lead` (enforced) |
| `scene_config` | `''` | the scene YAML; walls are projected here, before the leash |
| `contact_depth_full` | `0.05` | wall depth that reports as full-scale contact in `TeleopFeedback` |
| `publish_rate` | `100.0` | TF broadcast rate for `target_frame` |
| `require_teleop_mode` | `true` | ignore input unless the mode manager says `TELEOP` |

### `mode_manager` — arbitration, deadman, hardware gate

The single place that decides whether anything may move. Also the only real
deadman: a solver keeps driving toward its last target if input merely stops
arriving, so something must actively pause it.

| | |
|---|---|
| Subscribes | `input_topic` (`TeleopInput`), `event_topic` (`TeleopEvent`), `<servo_node>/status`, the measured joint states |
| Publishes | `~/status` (`TeleopStatus`, 20 Hz); a homing `JointTrajectory` on the arm controller |
| Serves | `~/set_mode` (`SetTeleopMode`) |
| Calls | the solver's pause service, servo's `ServoCommandType`, and `hardware_gate_service` (`SetBool`) |

| Parameter | Default | |
|---|---|---|
| `command_source` | `servo` | `ik` pauses `/ik_node/pause`; `servo` drives servo's command-type service |
| `ready_state` | `ready_pose` | SRDF named state homed to at startup |
| `goto_ready` | `true` | home before enabling the solver |
| `home_via_moveit` | `true` | plan homing through `move_group` so the path is collision-checked; `false` forces the direct interpolated move |
| `home_fallback` | `true` | allow the direct move when `move_group` refuses to plan. `false` means an unplannable home halts instead |
| `move_action` | `/move_action` | the `MoveGroup` action to plan through |
| `home_planning_time` / `home_planning_attempts` | `5.0` / `2` | planning budget |
| `home_scaling` | `0.3` | fraction of the model's velocity **and acceleration** limits for a planned posture move |
| `home_duration` / `home_timeout` / `home_tolerance` | `3.0` / `12.0` / `0.05` | the fallback trajectory's duration, the backstop deadline, and the arrival tolerance the fallback is judged by |
| `scene_ready_topic` / `scene_ready_timeout` | `''` / `10.0` | wait for `scene_objects` to report the scene applied before planning a path against it. Empty means nothing to wait for |
| `input_timeout` | `0.1` | deadman: older input than this halts |
| `watchdog_rate` / `status_rate` | `50.0` / `20.0` | Hz |
| `hardware_gate_enabled` | `false` | REQUIRED on real hardware — see below |
| `hardware_gate_service` | `/control_enable` | called `true` on engage, `false` on disengage |
| `hardware_gate_open_for_planning` | `true` | whether that gate is also open while a planned trajectory executes. `true` for a motion enable, `false` for a streaming-mode select whose driver refuses trajectories while it is active |

The hardware gate exists because a vendor gate that only opens for
`FollowJointTrajectory` goals never opens for a streamed command. Driving it
from here also makes it a real deadman on the CAN path. Launch the robot's own
bringup with its auto-gate disabled so the two do not fight.

### `gripper_node` — normalized grip → whatever gripper this robot has

Maps the device's 0…1 grip onto the SRDF gripper group's own open and closed
states, so no joint value appears in any config.

| | |
|---|---|
| Subscribes | `input_topic`, `status_topic`, `joint_states_topic` |
| Publishes | `<gripper controller>/joint_trajectory` |
| Parameters | `gripper_group` (`gripper`), `open_state`/`closed_state` (`''` → SRDF states matching *open*/*close*), `controller` (`''` → derived from the controller map), `rate` (20 Hz), `command_time` (0.15), `deadband` (0.01), `require_teleop_mode` (true) |

### `scene_objects` — publish the teleop scene into the planning scene

Reads the same `scene_config` the retargeter and solver read, and applies its
boxes as collision objects through `/apply_planning_scene`. Runs once at
startup.

### `joint_limit_guard` — clamp a streamed command to the model's limits

Sits between a command producer and the controller (`input_topic` →
`output_topic`), clamping each point to the URDF's position limits with
`margin`. Used on the **servo** path; `ik_node` enforces limits as constraints
inside the solve and needs no guard.

### `servo_session` — the deprecated servo path's session setup

Homes the arm and switches `moveit_servo` into pose-tracking mode, then holds.
Kept only as the A/B reference for `solver:=servo`; `mode_manager` does the same
work and more.

---

## Library modules

| Module | What it is for |
|---|---|
| `robot_info.py` | The only place that answers robot questions: joints, limits, tip link, planning frame, named states, controllers — read from `move_group`'s parameters, never re-parsed from xacro |
| `controllers.py` | Which ros2_control controller drives which joints |
| `scene_config.py` | Parse the scene YAML into collision boxes plus half-space walls `n·p ≥ d`; project a point onto them |
| `retargeting/base.py` | The plugin seam: `RetargetInput` → `RetargetOutput`. A bimanual rig or a dexterous hand adds a strategy by declaring the `moveit_teleop.retargeters` entry point from its own package |
| `retargeting/relative_pose.py` | The default strategy, including the anchor, the reference frames and every guard in the retargeting stage |
| `transforms.py` | Rigid-transform helpers, including the swing–twist split that extracts a device *heading* |
| `planner.py` | Plan-and-execute one joint-space goal through `move_group`'s `MoveGroup` action, non-blocking, plus the policy for which planner verdicts justify falling back to a direct move |
| `session.py` | Shared servo-handshake helpers |

## Tests

```bash
python3 -m pytest moveit_teleop_core/test -q
```

Covers the retargeting guards (anchor, jump refusal, wall re-anchor, leash),
the reference frames, wall projection and the scene parser — all without ROS.
