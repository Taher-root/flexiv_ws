# Configuration reference

Four layers, in the order you are likely to touch them:

1. [Robot config](#1-robot-config--teleopyaml) — `<arm>_moveit_config/config/teleop.yaml`, **36 keys**, plus the exporter's `dataset:` section
2. [Planning scene](#2-planning-scene--teleop_sceneyaml) — obstacles and virtual walls
3. [Recording](#3-recording--recordingyaml) — what an episode contains
4. [Session arguments](#4-session-arguments) — **28** launch arguments describing *this run*
5. [Node parameters](#5-node-parameters) — not exposed as launch arguments

Marked **REQ** = no sensible default for a new arm.
Marked **SAFETY** = bounds what a fault can do.

Defaults shown are the resolver's fallbacks, which apply when your file omits
the key. The Piper and FR3 columns are what the two shipped arms actually run.

---

## 1. Robot config — `teleop.yaml`

### Identity

| Key | Type | Default | Piper | FR3 | Notes |
|---|---|---|---|---|---|
| `move_group` **REQ** | string | `arm` | `arm` | `fr3_arm` | The SRDF planning group. Base link, tip link, joint order and the controller are all resolved from this one name. |
| `ready_state` **REQ** | string | `ready_pose` | `ready_pose` | `ready_pose` | SRDF `<group_state>` the arm homes to. Do not use all-zeros unchecked: on the Piper it is singular *and* sits on two joint limits. |
| `nominal_state` **REQ** | string | `ready_pose` | `ready_pose` | `ready_pose` | Posture the IK is regularised toward. `midrange` uses joint mid-range. The node prints this posture's Jacobian condition number at startup and warns above 500. |

### `files:` — companion paths, resolved relative to this file

| Key | Type | Default | Piper | FR3 | Notes |
|---|---|---|---|---|---|
| `files.scene` | path | `''` (none) | `teleop_scene.yaml` | `scene.yaml` | Collision boxes and virtual walls. Omit and the arm works in free space. |
| `files.rviz` | path | bringup's `teleop.rviz` | *(fallback)* | `teleop.rviz` | The fallback's TF whitelist names the Piper's `tcp_link`; a new arm should copy it and change two lines. |
| `files.preview_urdf` | path | `''` (no preview) | `agx_arm.urdf.xacro` | `fr3.urdf.xacro` | Model for the green ghost arm the replay panel scrubs episodes onto. |
| `files.preview_srdf` | path | `''` | `agx_arm.srdf.xacro` | `fr3.srdf.xacro` | Must name a `ready_pose`; `franka_description`'s own SRDF does not, which is why the FR3 ships a wrapper. |
| `files.preview_xacro_args` | string | `''` | arm_type, effector_type, tcp_offset | `''` | Must describe the **same** robot the bringup built, tool offset included, or the ghost is subtly the wrong shape. |

Omit all three `preview_*` keys and replay still works, without the ghost arm —
but **dataset export requires all three**: it runs offline, so it derives the
dataset's joint layout and units from these files rather than from a running
move_group.

### `retargeting:` — hand motion → tool pose

| Key | Type | Default | Piper | FR3 | Notes |
|---|---|---|---|---|---|
| `translation_scale` **REQ** | float | `0.5` | `1.5` *(ik)* | `1.5` *(ik)* | Device metres → robot metres. Above 1:1 is safe on the IK path: an unreachable target yields the closest *feasible* pose, so overshooting parks the tool at the workspace boundary instead of running away. |
| `rotation_scale` | float | `1.0` | `0.8` | `0.5` | Scaled independently of translation and in the opposite direction: a wrist has roughly the tool's range, so 1:1 is twitchy. |
| `translation_ref` | `base`&nbsp;\|&nbsp;`device`&nbsp;\|&nbsp;`tool` | `base` | `device` | `device` | `device` measures relative to the operator's horizontal heading at clutch engage, so "forward" is "away from me" however you are facing. `base` is the room, and turning around reverses every control. |
| `rotation_ref` | same three | `base` | `device` | `device` | `tool` composes a body-frame delta and needs a controller-to-tool alignment constant to feel right. |
| `smoothing_alpha` | float (0,1] | `1.0` | `0.3` | `1.0` | EMA weight on each new device sample; 1.0 disables it. 0.3 costs ~32 ms of phase at the Quest's 72 Hz. |
| `max_translation_step` **SAFETY** | m/update | `0.05` | `0.05` | `0.05` | A single update moving the target further than this is a teleport or a decoded garbage pose, and is refused. Applied to the *scaled* delta, so it limits robot motion. |
| `max_target_lead` **SAFETY** | m | `0.05` | `0.30` *(ik)* | `0.30` *(ik)* | The leash: how far the commanded target may lead the actual end effector. Loose on the IK path, tight on servo. |
| `max_target_lead_angle` | rad | `0.5` | `0.5` | `0.5` | Same leash for orientation. |
| `external_motion_threshold` | m | `0.15` | `0.50` *(ik)* | `0.50` *(ik)* | Unexplained EE motion that forces a re-anchor. **Must exceed `max_target_lead`** or ordinary lag reads as a teleport and the arm re-anchors continuously; the retargeter raises at construction if it does not. |
| `max_command_step` | m/update | `0.03` | `0.03` | `0.03` | How far a *wall* may move the command beyond what the hand drove, before the mapping is re-zeroed and the arm holds. `0` disables. |
| `per_solver.{ik,servo}.*` | mapping | — | 4 keys | 4 keys | Overrides any key above for one solver. |

### `ik:` — the solver

| Key | Type | Default | Piper | FR3 | Notes |
|---|---|---|---|---|---|
| `solve_rate` | Hz | `90.0` | `90.0` | `90.0` | Lower it if `/ik_node/status` reports `solve_time_p99` approaching the frame period. IQ9075: median 2.77 ms, p99 4.3 ms against 11.1 ms. |
| `weight_position` | float | `50.0` | `50.0` | `50.0` | The reference term; leave it and move the others relative to it. |
| `weight_orientation` | float | `10.0` | `0.5` | `10.0` | Raise for tight tool alignment, lower to stop the wrist swinging on simple translations. See [tuning](tuning.md#4-fix-the-wrist-with-weight_orientation). |
| `weight_regularization` | float | `0.005` | `0.005` | `0.02` | Pull toward `nominal_state`, and the *sole* source of steady-state tracking error. Set by DOF, not taste. |
| `weight_smoothness` | float | `0.1` | `0.1` | `0.1` | Penalises departure from the previous solution. Raise if the arm chatters between nearby IK solutions. |
| `rate_limit_scale` **REQ SAFETY** | float (0,1] | `0.3` | `0.5` | `1.0` | Fraction of each joint's MoveIt maximum velocity teleop may use. Your main speed ceiling. |
| `max_joint_jump` **SAFETY** | rad | `0.35` | `0.35` | `0.35` | Refuse a solution further than this from the *previous solution* — an IK branch flip is valid kinematics and unacceptable motion. |
| `jump_retry_smoothness` | multiplier | `50.0` | `50.0` | `50.0` | On a refused jump, retry once with smoothness multiplied by this. Measured: 17 refusals → 0. `0` restores plain refusal. |
| `collision_policy` **SAFETY** | `stop`&nbsp;\|&nbsp;`none` | `stop` | `stop` | `stop` | `stop` runs at full speed to contact and refuses to enter a collision, holding the last valid command. |

### `gripper:` / `controllers:` / `safety:`

| Key | Type | Default | Piper | FR3 | Notes |
|---|---|---|---|---|---|
| `gripper.enabled` | bool | `false` | `true` | `true` | Run the gripper node. |
| `gripper.group` | string | `gripper` | `gripper` | `fr3_hand` | Its open/closed joint values are *not* configured — they come from that group's SRDF states matching `*open*` and `*close*`. |
| `controllers.arm` **REQ** | string | `arm_controller` | `arm_controller` | `fr3_arm_controller` | Genuinely not recorded in any MoveIt artifact, which is why it is here. |
| `safety.input_timeout` **SAFETY** | s | `0.1` | `0.1` | `0.1` | Deadman. Measured on the device's own `header.stamp`, not arrival — a driver republishing a frozen pose is invisible to an arrival check. |
| `safety.hardware_gate.enabled` **SAFETY** | bool | `false` | `false` | `false` | **Required on real hardware** where a gate exists: it is what makes the deadman reach the CAN bus. |
| `safety.hardware_gate.service` | string | `/control_enable` | same | same | The `std_srvs/SetBool` service. Launch the robot bringup with `auto_control_gate:=false` so the two do not fight. |
| `safety.hardware_gate.open_for_planning` **SAFETY** | bool | `true` | `true` | `true` | Whether that gate must also be open while `move_group` executes a planned trajectory. `true` for a gate that **enables motion** (the Piper's CAN gate). `false` for one that **selects a streaming mode** the driver's own trajectory action refuses to run alongside — the Flexiv driver rejects the goal, which reads as a plan failure and silently falls back forever. |
| `safety.home_via_moveit` **SAFETY** | bool | `true` | `true` | `true` | Plan homing through `move_group`, so its path is collision-checked and time-parameterised against `joint_limits.yaml`. `false` uses the direct interpolated move — which is also the automatic fallback when `move_group` cannot plan one. A per-robot capability question: every MoveIt config with a controller manager can execute. |

### `dataset:` — dataset export (read by `moveit_teleop_record convert`)

The teleop launch ignores this section; the exporter validates it and refuses an
unknown key. All optional. Which joints are the arm and gripper, their order and
their units are **not** configured — they come from `move_group`, `gripper.group`
and the `files.preview_*` model. See
[moveit_teleop_record/README.md](../../moveit_teleop_record/README.md).

| Key | Type | Default | Piper | FR3 | Notes |
|---|---|---|---|---|---|
| `dataset.robot_type` | string | URDF name | `agx_piper` | `franka_fr3` | LeRobot `info.json` robot_type. |
| `dataset.image_size` | `WxH` \| `native` | `640x360` | `640x360` | `640x360` | Both GR00T and π0.5 resize to 224. |
| `dataset.vcodec` | `h264` \| `libsvtav1` | `h264` | same | same | h264 plays in any player; AV1 is about 2/3 the size and matches GR00T's demo data, but VLC 3.0 would not play it. |
| `dataset.state_stream` | string | `joint_states` | same | same | Recording stream name → `observation.state`. |
| `dataset.action_stream` | string | `joint_commands` | same | same | Recording stream name → `action`. |
| `dataset.trim` | `teleop` \| `none` | `teleop` | same | same | First engage → last disengage. |
| `dataset.gr00t.action_horizon` | int | `16` | `16` | `16` | Steps at the camera's fps. |
| `dataset.gr00t.arm_action` | `relative` \| `absolute` | `relative` | same | same | In the generated GR00T config. |
| `dataset.gr00t.gripper_action` | `relative` \| `absolute` | `absolute` | same | same | |

### `device:` — the scripted device only (ignored when `device:=xr`)

| Key | Type | Default | Notes |
|---|---|---|---|
| `device.pattern` | string | `circle` | `circle`, `square`, `ramp`, `wobble`, `clutch_storm`, `dropout`, `static`. The last two exist to break things. |
| `device.period` | s | `6.0` | Seconds per cycle. |
| `device.radius` | m | `0.1` | Pattern size in device metres, before `translation_scale`. |

---

## 2. Planning scene — `teleop_scene.yaml`

One file, two mechanisms, because they are two views of the same obstacle — and
a box's wall is **derived** from that box's bounds so the two cannot drift.

| Key | Type | Default | Notes |
|---|---|---|---|
| `planning_scene.default_frame` | string | `''` | Frame for entries that name none. |
| `boxes[].name` **REQ** | string | — | Planning-scene object id. Duplicates are refused: the second would silently replace the first. |
| `boxes[].frame` | string | `default_frame` | Anchor to a robot **link**, not the planning frame: geometry on `base_link` stays put when the robot is re-mounted. |
| `boxes[].bounds` **REQ** | 6 floats | — | `[xmin, xmax, ymin, ymax, zmin, zmax]`. Measure it. A table top that reaches the robot base intersects it, every state is then in collision, and the arm never moves. |
| `boxes[].collision` | bool | `true` | Publish as a `CollisionObject` the checker and MoveIt's planner both see. |
| `boxes[].wall` | `±x ±y ±z` | `''` | Which face becomes an IK constraint. `+z` keeps the tool above the upper z face — and the tool *slides* along it, because the solver returns the closest feasible pose. |
| `boxes[].wall_clearance` **SAFETY** | m | `0.0` | How far off the face the wall sits. Must cover **tool geometry and tracking error**. See [tuning step 5](tuning.md#5-size-wall_clearance-against-tracking-error). |
| `boxes[].wall_bounded` | bool | `false` | Leave false. A vanishing constraint moves the optimum discontinuously — measured 218 mm in one solve at the footprint edge. |
| `boxes[].wall_margin` | m | `0.0` | Extra footprint beyond the box sides, when bounded. |
| `planes[].name` **REQ** | string | — | A wall with no matching geometry. |
| `planes[].normal` **REQ** | 3 floats | — | Enforces `n · p ≥ offset` on the tool point. Normalised for you. |
| `planes[].offset` | m | `0.0` | The half-space constant. |
| `planes[].extent` | 4 floats | none | Optional footprint along the two axes perpendicular to the normal. Only axis-aligned normals may carry one — for a tilted plane those axes would be a guess, and that is refused. |

---

## 3. Recording — `recording.yaml`

Session-wide rather than per-robot: it describes what a training sample
*contains*. Session-dependent topics are written as `${placeholder}` and filled
by the launch.

| Key | Type | Default | Notes |
|---|---|---|---|
| `recording.output_dir` | path | `~/teleop_episodes` | Where episodes are written. **Also where the replay panel looks** — `episodes_dir` is derived from this so the two cannot disagree. |
| `recording.storage_id` | string | `mcap` | rosbag2 storage plugin. |
| `recording.discard_on_halt` | bool | `true` | An episode during which the watchdog fired is a demonstration in which the operator lost control. |
| `streams[].topic` **REQ** | string | — | May contain `${target_topic}`, `${input_topic}`, `${solver_status_topic}`. An unknown placeholder is an error, never a pass-through. |
| `streams[].role` **REQ** | enum | — | `observation` \| `action` \| `context` \| `camera` \| `sensor`. Frozen into the sidecar so the exporter needs no re-description. |
| `streams[].name` | string | from topic | Stable name in the exported dataset. |
| `streams[].required` | bool | `false` | A missing required stream **refuses the episode** rather than recording a hole. |
| `streams[].fields` | list | whole msg | Top-level members to keep at export. |
| `export.rate_hz` | Hz | `30.0` | Resample rate. Bags hold 100–200 Hz raw, so this can be raised later without re-recording. |
| `export.max_staleness` | s | `0.05` | A frame is emitted only when *every* required stream has a sample within this. |
| `export.default_interpolation` | enum | `zero_order_hold` | `zero_order_hold` for commands and anything discrete; `linear` only for continuous quantities. |

---

## 4. Session arguments

28 launch arguments describing *this run* rather than the robot, so they stay on
the command line and are deliberately not in the config file.

| Argument | Default | Notes |
|---|---|---|
| `robot_config_package` | `''` | The arm's MoveIt config package; reads `<share>/config/teleop.yaml`. The normal way to select a robot. |
| `robot_config` | from package | Explicit path, for a file not in a package yet. Wins over the package. |
| `device` | `xr` | `xr` starts the Quest WebXR bridge; `fake` runs the scripted device. |
| `hand` | `right` | Which Quest controller drives the arm. |
| `solver` | `ik` | `ik` is the supported path. `servo` is **deprecated** and untuned. |
| `joint_states_topic` | `/control/joint_states` | What the solver, mode manager and gripper treat as robot state. **On real hardware point this at the driver's feedback topic.** |
| `description_source` | `/move_group` | Node whose parameters carry the URDF and SRDF. |
| `goto_ready` | `true` | Home to `ready_state` before enabling the solver. |
| `record` | `false` | Run the episode recorder. *This is the one people forget.* |
| `record_dir` | `''` | Override `recording.output_dir` for this run. |
| `recording_config` | bringup's | Which `recording.yaml` to use. |
| `task` | `''` | Task label written into every episode sidecar. |
| `use_rviz` | `true` | Start RViz with the teleop view and the replay panel. |
| `replay` | `true` | Add the episode player and preview robot. Skipped automatically when the config names no preview xacro. |
| `episodes_dir` | from `recording.yaml` | Where the panel looks. Override only to review an archive. |
| `autoload_latest` | `false` | Open the newest episode at startup. Off, so a live session does not open with a ghost arm on the real one. |
| `feedback_topic` | `/teleop/feedback` | Wall-contact level the XR bridge turns into a controller buzz. |
| `input_topic`, `event_topic` | from `device` | Derived, so nobody keeps them in sync by hand. |
| `device_convention` | from `device` | `webxr` or `ros`. Axis conversion lives in the retargeter, in exactly one place. |
| `servo_node` | `/servo_node` | Node the mode manager pauses. Automatically `/ik_node` when `solver:=ik`. |
| `launch_servo` | `true` | Set false if a `servo_node` is already running. |
| `xacro_mappings` | `''` | **Deprecated servo path only**: servo builds its own model, so it needs the bringup's xacro args exactly. Unused when `solver:=ik`. |
| `pattern`, `period`, `radius` | from config | Scripted-device overrides. |
| `plane` | `xy` | Scripted-device sweep plane. `xz` is the only way it can reach a table. |
| `offset`, `offset_ramp` | zeros, `3.0` | Scripted-device pattern centre and ramp-in time. |
| `start_delay` | `0.0` | Hold still this long before moving. Must outlast homing, or the motion is absorbed into the retargeting anchor. |
| `emit_episode_events` | `false` | Have the fake device drive a record/save cycle, for testing the recorder unattended. |

---

## 5. Node parameters

Not exposed as launch arguments, because changing them is rare and usually means
something else is wrong. Set with `--ros-args -p` or by editing the node's
launch include.

| Node · parameter | Default | Notes |
|---|---|---|
| `ik_node.max_iter` | `30` | IPOPT iteration cap. Hitting it is normal and yields status `acceptable`, which is still used. |
| `ik_node.limit_margin` | `0.01` | Rad kept inside the URDF joint limits, so solutions do not land exactly on a bound. |
| `ik_node.smoothing_weights` | `[0.4,0.3,0.2,0.1]` | Post-solve weighted moving average, newest first. Renormalised over however many samples exist. |
| `ik_node.use_moveit_limits` | `true` | Prefer MoveIt's `joint_limits.yaml` over the URDF. Streaming commands must obey the same ceiling as planned motions. |
| `ik_node.use_moveit_velocity_scaling` | `false` | Also fold in `default_velocity_scaling_factor`. Off, because that is a *planning* default, not a safety ceiling. |
| `ik_node.target_timeout` | `0.5` | Stop commanding if no fresh target arrives. The mode manager owns the actual halt. |
| `ik_node.collision_check_rate` | `20.0` | Validity-service rate. Note the gap against a 90 Hz solve: up to four commands can be published before it reacts. |
| `ik_node.command_time` | `0` → one period | `time_from_start` on each published point. |
| `ik_node.planes` | `[]` | Flat `[nx,ny,nz,offset,...]` walls in addition to the scene file's. Always active; no footprint. |
| `mode_manager.watchdog_rate` | `50.0` | How often staleness is evaluated. |
| `mode_manager.home_duration` | `3.0` | Seconds allotted to the homing trajectory. |
| `mode_manager.home_timeout` | `12.0` | Give up and HALT. On hardware, check the CAN gate and drag-teach mode first. |
| `mode_manager.home_tolerance` | `0.05` | Rad per joint that counts as arrived. |
| `retarget_node.publish_rate` | `100.0` | Output rate, decoupled from the device's own jittery rate. |
| `retarget_node.retargeter` | `relative_pose` | Entry point under `moveit_teleop.retargeters`. A different strategy is a plugin, not a fork. |
| `retarget_node.ee_link` | `''` → SRDF | Override the end-effector link. Needed only when a group has several. |
| `retarget_node.contact_depth_full` | `0.05` | Depth past a wall at which haptic intensity reaches 1.0. |
| `gripper_node.open_state`, `.closed_state` | `''` → search | Override the SRDF state search for `*open*` / `*close*`. |
| `gripper_node.deadband` | `0.01` | Command change below which no new trajectory is sent. |
| `xr_device.port`, `.certfile`, `.keyfile` | `8443`, required | WebXR needs a secure context. Generate a pair once per host with `make_cert.sh <lan-ip>`. |
| `xr_device.clutch_threshold` | `0.5` | Squeeze value at which the clutch engages. |
| `xr_device.gripper_deadzone` | `0.03` | Below this the trigger reads as fully released; Quest triggers rest above zero. |
| `xr_device.haptics_enabled` | `true` | Forward wall contact to the controller as a rumble. |
| `episode_player.preview_color` | `[0.15,0.8,0.35,1]` | The ghost arm's colour, which is what makes a replay read as a preview. |
| `tf_prefixer.prefix`, `.root_frame` | `replay/`, `world` | The replay tree is prefixed so it cannot collide with the live robot's frames; the root keeps its name so both share a world. |
