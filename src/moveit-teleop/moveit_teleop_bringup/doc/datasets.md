# Datasets: record, export, fine-tune

Teleop episodes become a **LeRobot v2.1** dataset that both **GR00T** and
**π0.5** (openpi) fine-tune on, with every stream aligned to the camera's capture
time.

The exporter is robot-agnostic. Its setup, bringup, and the exact list of what a
robot's config must provide are in
[moveit_teleop_record/README.md](../../moveit_teleop_record/README.md); this page
walks the Piper through it end to end.

```
kit:      arm  +  camera  +  teleop task:="..."      ->  MCAP episodes
desktop:  ros2 run moveit_teleop_record convert      ->  LeRobot v2.1 + modality.json
GPU box:  GR00T launch_finetune.py  |  openpi train.py
```

---

## 1. Record (on the kit)

```bash
arm                                  # terminal 1, wait for controllers
camera                               # terminal 2
teleop task:="pick up the block"     # terminal 3
```

**A** starts and stops (saving), **B** discards, squeeze to engage. Each take is one
episode in `~/Downloads/sample_teleop_episodes/`.

What is recorded, and what it becomes:

| Stream | Topic | In the dataset |
|---|---|---|
| `joint_states` | `/feedback/joint_states` — measured, 200 Hz | `observation.state` |
| `joint_commands` | `/control/joint_states` — commanded, 200 Hz | `action` |
| `gripper_camera` | `…/color/image_raw/compressed` — 15 Hz | `observation.images.gripper_camera` |
| `teleop_status` | `/mode_manager/status` | trims the episode |
| task | `teleop task:="..."` | `task_index` / language |

Watch `teleop`'s output as you start: if pressing A prints
`recording WITHOUT camera(s)`, the camera is not running and that take has no
images.

Press **home** before stopping if you like — the export trims it off.

## 2. Export (on the desktop)

Once, create the pinned LeRobot environment:

```bash
bash $(ros2 pkg prefix moveit_teleop_record)/share/moveit_teleop_record/scripts/setup_lerobot_env.sh
```

Then pull episodes off the kit and convert, naming the robot exactly as the
teleop launch does:

```bash
rsync -a ubuntu@<kit>:Downloads/sample_teleop_episodes/ ~/teleop_episodes/
ros2 run moveit_teleop_record convert --robot-config-package agx_arm_moveit \
    ~/teleop_episodes ~/datasets/piper_pick
ros2 run moveit_teleop_record view ~/datasets/piper_pick     # http://127.0.0.1:9090
```

The joint layout is derived from the robot's SRDF, and printed first:

```
robot 'agx_arm' from .../agx_arm_moveit/share/agx_arm_moveit/config/teleop.yaml
  arm      [0:6]  SRDF 'arm': joint1 (rad), ..., joint6 (rad)
  gripper  [6:7]  SRDF 'gripper': gripper (m)
```

Add later sessions with `--append`. Every episode prints its alignment report:

```
  20260914_145738_ep1789397858
     373/506 frames at 15 fps, teleop 3.92->28.77 s of 33.63 s
     camera latency 30.5 ms (corrected: aligned on capture time), joints bracket every frame within 10.93 ms
     image-motion vs joint-speed: best +10 ms, r=0.655, consistent with -60..+92 ms -> aligned
  ...
  all episodes pooled (787 frame intervals): best -6 ms, r=0.617, consistent with -70..+60 ms -> aligned
```

…and the dataset is then **loaded back with openpi's exact lerobot** and checked
before `convert` reports success.

| Option | Default | |
|---|---|---|
| `--append` / `--overwrite` | refuse if the dataset exists | never writes into a directory that isn't a dataset |
| `--image-size` | `640x360` | both models resize to 224; `native` keeps 1280×720 |
| `--vcodec` | `h264` | plays in any player; `libsvtav1` (AV1) is about 2/3 the size and matches GR00T's demo data, but VLC 3.0 would not play it |
| `--trim` | `teleop` | first engage → last disengage; `none` keeps everything |
| `--strict` | off | fail instead of skipping a bad episode |
| `--task "…"` | as recorded | one task string for all episodes, e.g. to merge `"move iron"` and `"move-iron"` |

Defaults come from the robot config's `dataset:` section; a flag overrides it.

### How the time sync works

Every rule came from measuring a real episode.

- **Aligned on capture time, not receive time.** The camera's receive time lags
  its capture by 30.6 ± 0.7 ms; the arm's by 4.7 ms. Receive-time pairing would
  give every image the arm state from ~26 ms later.
- **The camera drives the timeline.** One dataset frame per real camera frame; the
  200 Hz joint streams are interpolated to each frame's capture time.
- **Joints matched by name.** Measured state lists `joint1..joint6, gripper`; the
  command stream lists `gripper, joint1..joint6`.
- **Repeated stamps removed.** The driver republishes an unchanged sample with
  the old stamp (0.8% of samples) — interpolating across it divides by zero.
- **Refuse, don't repair.** A camera gap, or a frame with no joint sample within
  50 ms, skips the episode with a reason rather than holding a stale value.
- **Measured, every export.** Image motion from a gripper camera is driven by arm
  motion, so cross-correlating the two estimates the residual offset. At 15 fps
  that estimate is broad — about ±100 ms per episode — so it detects gross errors
  (a camera on its own clock) rather than measuring fine ones. An episode is
  `MISALIGNED` only when its whole uncertainty band sits more than half a frame
  from zero; the episodes are also pooled into one session verdict. On the Piper
  session: `-6 ms` pooled, with zero inside the band.

LeRobot's timestamps are nominal (`index / fps`); the camera runs at 15.016 Hz,
so they drift from wall time by ~1 ms per second, which training never sees.

## 3. Fine-tune

The dataset layout is file-for-file that of GR00T's own demo dataset:

```
meta/info.json  episodes.jsonl  episodes_stats.jsonl  tasks.jsonl  modality.json
data/chunk-000/episode_000000.parquet
videos/chunk-000/observation.images.gripper_camera/episode_000000.mp4
```

| Feature | Shape | Names |
|---|---|---|
| `observation.state` | 7 | `joint1..joint6` (rad), `gripper` (m) — measured, layout from the SRDF |
| `action` | 7 | same order — commanded |
| `observation.images.gripper_camera` | 360×640×3 | h264, 15 fps |

### GR00T

Each export writes `gr00t_modality_config.py` into the dataset, generated from the
robot's groups and cameras and its `dataset.gr00t` settings:

```bash
uv run python gr00t/experiment/launch_finetune.py \
    --base-model-path nvidia/GR00T-N1.7-3B \
    --dataset-path ~/datasets/piper_pick \
    --embodiment-tag NEW_EMBODIMENT \
    --modality-config-path ~/datasets/piper_pick/gr00t_modality_config.py \
    --num-gpus 1 --output-dir /tmp/piper_gr00t --max-steps 2000
```

By default the arm action is relative and the gripper absolute, with 16 steps of
horizon — the same choices as GR00T's SO-100 reference config. At 15 fps that is
1.07 s of lookahead. GR00T writes `stats.json` itself on first use.

**Verified against GR00T's own data code** (loader, stats generator, torchcodec),
not just its documentation — including that it slices state and action correctly,
which it silently does not do if a parquet cell reads as a list:

```bash
bash $(ros2 pkg prefix moveit_teleop_record)/share/moveit_teleop_record/scripts/setup_gr00t_check_env.sh
env -i HOME=$HOME PATH=/usr/bin:/bin ~/miniconda3/envs/gr00t_dataload/bin/python \
    $(ros2 pkg prefix moveit_teleop_record)/share/moveit_teleop_record/training/verify_gr00t.py \
    --gr00t ~/Isaac-GR00T --dataset ~/datasets/piper_pick
```

### π0.5 (openpi)

The dataset loads with the exact lerobot commit openpi pins (`0cf8648`), with the
same `huggingface-hub`, `datasets`, `pyarrow` and `torchcodec` as openpi's lockfile,
including 16-step action chunks with padding at episode ends.

openpi has no generic dataset flag: a new robot needs a `TrainConfig` and a policy
input/output transform in openpi's source, mapping `observation.images.gripper_camera`,
`observation.state`, `action` and `task` to the model's inputs, followed by
`scripts/compute_norm_stats.py` and `scripts/train.py`. That config is **not yet
written or verified** here.
