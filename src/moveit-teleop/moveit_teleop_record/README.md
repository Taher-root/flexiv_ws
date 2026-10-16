# moveit_teleop_record

Record teleoperation episodes, export them as a **LeRobot v2.1** dataset that
**GR00T** and **π0.5** (openpi) fine-tune on, and look at the result.

```
record    teleop record:=true      ->  one MCAP episode per take
export    ros2 run ... convert     ->  LeRobot v2.1 + GR00T modality config
view      ros2 run ... view        ->  LeRobot's viewer: video synced with joints
```

Nothing in this package names a robot. An export reads the same robot config the
teleop launch does, and derives the dataset's joint layout from that robot's
MoveIt model.

Signal flow for both halves:
[moveit_teleop_bringup/doc/architecture.md](../moveit_teleop_bringup/doc/architecture.md#6-recording-and-export).

---

## API

### Node: `episode_recorder`

Subscribes to every stream `recording.yaml` declares and writes raw messages at
their native rates with their original timestamps. It interprets nothing beyond
"subscribe and write": recording raw is lossless and cheap, and the same bags
can be re-exported at a different rate or with a different stream set later. You
cannot un-sync a bag that was resampled on the way in.

| | |
|---|---|
| Subscribes | `event_topic` (`TeleopEvent`: save / discard), `status_topic` (`TeleopStatus`), plus every stream in the config |
| Writes | one MCAP bag and one `episode.json` per episode, under `output_dir` |

| Parameter | Default | |
|---|---|---|
| `config_file` | `''` | the recording config. `moveit_teleop_bringup/config/recording.yaml` is the one the launch passes |
| `output_dir` | `''` | `''` uses the config's `output_dir` |
| `task` | `''` | the language instruction stored with every episode in this run |
| `event_topic` / `status_topic` | device-dependent / `/mode_manager/status` | |
| `topic_substitutions` | `['']` | `name=topic` pairs filling the config's `${...}` references — which topic carries the action, the input and the solver status depends on how the stack was launched, not on the robot |

`discard_on_halt: true` in the config throws away an episode during which the
watchdog fired: that is a demonstration in which the operator lost control, and
keeping it quietly poisons a training set.

### Command-line tools

| | |
|---|---|
| `ros2 run moveit_teleop_record convert` | episodes → a LeRobot v2.1 dataset (see below) |
| `ros2 run moveit_teleop_record view <dataset>` | LeRobot's HTML viewer on `http://127.0.0.1:9090` |

### Library

| Module | |
|---|---|
| `config.py` | parse `recording.yaml`; stream names, roles and `${...}` substitution |
| `reader.py` | `Episode`: read a bag and resample it onto a common timeline |
| `sync.py` | pure-numpy time alignment: dedupe repeated stamps, interpolate, find gaps, choose the fps, and cross-correlate image motion against joint speed |
| `extract.py` | one episode → one aligned `Bundle`, with its sync report |
| `robot_model.py` | the dataset's joint layout and units, derived from the arm's URDF and SRDF |
| `lerobot_writer.py` | bundles → dataset. Runs in the pinned LeRobot environment and **imports nothing from ROS** — a test asserts that against its AST |
| `exporters.py` | flatten ROS messages into numeric features |

---

## Setup — once per machine

Exporting needs two Python environments that cannot share an interpreter: ROS's,
which reads the bags, and a pinned LeRobot, which writes the dataset. `convert`
runs the second one for you with ROS stripped from its environment.

```bash
# 1. The ROS workspace, built, with the robot's MoveIt config package in it.
source /opt/ros/jazzy/setup.bash
source ~/<workspace>/install/setup.bash

# 2. The LeRobot environment: openpi's pinned lerobot (commit 0cf8648, dataset
#    format v2.1), with the rest pinned to openpi's uv.lock. CPU only. ~5 min.
bash $(ros2 pkg prefix moveit_teleop_record)/share/moveit_teleop_record/scripts/setup_lerobot_env.sh
```

It creates the conda env `lerobot_v21`, which `convert` finds automatically.
Anywhere else: `--lerobot-python /path/to/python` or `LEROBOT_PYTHON`.

Optional — a small environment that runs **GR00T's own data loader** on a dataset,
to check it before spending GPU time (no flash-attn, no CUDA):

```bash
bash $(ros2 pkg prefix moveit_teleop_record)/share/moveit_teleop_record/scripts/setup_gr00t_check_env.sh ~/Isaac-GR00T
```

---

## Bringup

### Record

On the robot's computer, with the camera running:

```bash
ros2 launch moveit_teleop_bringup teleop.launch.py \
    robot_config_package:=<arm>_moveit_config record:=true task:="pick up the block"
```

A saves, B discards. Episodes land in `recording.yaml`'s `output_dir`.

### Export

```bash
ros2 run moveit_teleop_record convert \
    --robot-config-package <arm>_moveit_config \
    ~/teleop_episodes  ~/datasets/my_task
```

| Flag | |
|---|---|
| `--robot-config-package` / `--robot-config` | **Required.** The robot, named exactly as for teleop. |
| `--append` | Add episodes to an existing dataset. |
| `--overwrite` | Replace one. Without either flag, an existing dataset is refused, and a non-dataset directory is never written into. |
| `--task "…"` | One task string for every episode, overriding what was recorded. |
| `--image-size`, `--vcodec`, `--trim`, `--state-stream`, `--action-stream` | Override the robot config's `dataset:` section for this run. |
| `--strict` | Fail instead of skipping an episode that cannot be aligned. |
| `--keep-bundles` | Keep the intermediate aligned bundles. |

Each episode prints an alignment report, and the finished dataset is **loaded back
with openpi's lerobot and checked** — every state/action row against its source,
video frames against their rows, action chunking, timestamps — before `convert`
reports success.

### View

```bash
ros2 run moveit_teleop_record view ~/datasets/my_task        # open http://127.0.0.1:9090
```

LeRobot's own HTML viewer: each episode's video, with every state and action
dimension plotted underneath and a cursor that follows playback. Ctrl-C stops it.

---

## What a robot must provide

The exporter reads the robot's `teleop.yaml` — `<arm>_moveit_config/config/teleop.yaml`,
the same file the teleop launch reads.

### Derived — never configure these

| | From |
|---|---|
| Arm joints, in order | SRDF group `move_group`, walked base → tip |
| Gripper joints | SRDF group `gripper.group`, minus mimic joints |
| Units | URDF joint type: revolute/continuous → rad, prismatic → m |
| Robot name | URDF `<robot name>` |
| Dataset layout | arm group, then gripper group: `[0:6]` + `[6:7]` on a Piper, `[0:7]` + `[7:8]` on an FR3 |

### Required — the export stops, naming the key, without them

| Key in `teleop.yaml` | Why it cannot be derived |
|---|---|
| `move_group` | which SRDF group is the arm |
| `gripper.group` *(if `gripper.enabled`)* | which SRDF group is the gripper |
| `files.preview_urdf` | the robot description (xacro or URDF). An export runs **offline** — there is no move_group to ask — so the model has to come from a file. |
| `files.preview_srdf` | the semantic description (xacro or SRDF) |
| `files.preview_xacro_args` | the arguments the robot bringup gave xacro (tool offset included). `''` if none. |

File paths are relative to `teleop.yaml`. The replay panel reads the same three
`files.preview_*` entries, so a robot set up for replay is already set up for
export.

### Optional — the `dataset:` section

```yaml
moveit_teleop:
  dataset:
    robot_type: agx_piper          # LeRobot robot_type; default: URDF robot name
    image_size: 640x360            # WxH or native
    vcodec: h264                   # plays anywhere; libsvtav1 ~2/3 the size
    state_stream: joint_states     # recording.yaml stream names
    action_stream: joint_commands
    trim: teleop                   # or none
    gr00t:
      action_horizon: 16           # steps, at the camera's fps
      arm_action: relative         # relative | absolute
      gripper_action: absolute
```

The teleop launch ignores this section; the exporter validates it and refuses a
key it does not know.

### From the recording (session, not robot)

`recording.yaml` must record, by stream name: `joint_states` (measured, the
state), `joint_commands` (commanded, the action — optional on mock hardware, where
the command *is* the state), `teleop_status` (for trimming), and at least one
stream with `role: camera`. The first camera drives the timeline.

---

## Adding a new arm

1. Its `teleop.yaml` already exists if the arm is set up for teleop. Check
   `files.preview_urdf`, `files.preview_srdf` and `files.preview_xacro_args` point
   at its model — see `moveit_teleop_bringup/config/samples/template_teleop.yaml`.
2. Run `convert --robot-config-package <arm>_moveit_config` on one episode. The
   first thing it prints is the layout it derived:
   ```
   robot 'fr3' from .../franka_teleop_bringup/share/franka_teleop_bringup/config/teleop.yaml
     arm      [0:7]  SRDF 'fr3_arm': fr3_joint1 (rad), ..., fr3_joint7 (rad)
     gripper  [7:8]  SRDF 'fr3_hand': fr3_finger_joint1 (m)
   ```
   If that is wrong, the SRDF groups are wrong — fix them there, not here.
3. Optionally add a `dataset:` section.

No file in this package changes.

---

## The dataset

```
my_task/
  README.md                  what is in it, alignment per episode, how to train
  gr00t_modality_config.py   GR00T's --modality-config-path, generated for this robot
  meta/ info.json  episodes.jsonl  episodes_stats.jsonl  tasks.jsonl
        modality.json        GR00T's layout file
        moveit_teleop_export.json   robot, settings, and sync report per episode
  data/chunk-000/episode_000000.parquet
  videos/chunk-000/observation.images.<camera>/episode_000000.mp4
```

| Feature | Contents |
|---|---|
| `observation.state` | measured joint positions, arm then gripper |
| `action` | commanded joint positions, same order |
| `observation.images.<camera>` | one real camera frame per row |

This is the layout of GR00T's own demo dataset, file for file.

### Time alignment

- **Capture time, not receive time.** Measured on a Gemini 335L: images arrive
  30.6 ms after capture, joint feedback 4.7 ms. Receive-time pairing would give
  every image the arm state from ~26 ms later.
- **The camera drives the timeline**; 200 Hz joints are interpolated to each
  frame's capture time.
- **Joints by name** — a driver's publish order is not the robot model's.
- **Repeated stamps removed** before interpolating.
- **Refuse, don't repair**: a camera gap or a joint dropout over 50 ms skips the
  episode.
- **Trimmed** from first engage to last disengage, dropping idle and any HOME,
  where the planner — not the operator — drove the arm.

Every export also **measures** alignment: on a camera that moves with the arm,
image motion follows arm motion, so cross-correlating them estimates the residual
offset. At 15 fps that estimate is broad — about ±100 ms per episode — so it is a
detector for gross errors (a camera on its own clock, or stamping on arrival), not
a fine measurement. An episode is called MISALIGNED only when its whole
uncertainty band sits more than half a frame from zero, and the episodes are
pooled into one session verdict.

---

## Training

### GR00T

```bash
uv run python gr00t/experiment/launch_finetune.py \
    --base-model-path nvidia/GR00T-N1.7-3B \
    --dataset-path ~/datasets/my_task \
    --embodiment-tag NEW_EMBODIMENT \
    --modality-config-path ~/datasets/my_task/gr00t_modality_config.py
```

GR00T computes `meta/stats.json` itself. Check a dataset with GR00T's own loader
first:

```bash
env -i HOME=$HOME PATH=/usr/bin:/bin ~/miniconda3/envs/gr00t_dataload/bin/python \
    $(ros2 pkg prefix moveit_teleop_record)/share/moveit_teleop_record/training/verify_gr00t.py \
    --gr00t ~/Isaac-GR00T --dataset ~/datasets/my_task
```

### π0.5 (openpi)

The dataset loads with openpi's pinned lerobot. openpi has no generic dataset
flag, though: a new robot needs a `TrainConfig` and a policy input/output
transform in openpi's source, then `scripts/compute_norm_stats.py` and
`scripts/train.py`. **Not provided here yet.**

---

## Troubleshooting

| Message | Meaning |
|---|---|
| `` `files.preview_urdf` is REQUIRED to export datasets `` | The robot's teleop.yaml has no model files. See *What a robot must provide*. |
| `'joint_states' does not carry [...]` | The episode was recorded on a different robot than `--robot-config-package` names. |
| `no LeRobot interpreter at ...` | Run `setup_lerobot_env.sh`, or pass `--lerobot-python`. |
| `the camera skipped N time(s)` | Frames were dropped while recording; the episode is skipped. |
| `NOTE: episodes carry different task strings` | e.g. `"move iron"` and `"move-iron"` — each becomes a separate instruction. Re-run with `--task`. |
| `MISALIGNED` | Camera and joint clocks disagree by more than half a frame. Check the camera's `time_domain`. |
