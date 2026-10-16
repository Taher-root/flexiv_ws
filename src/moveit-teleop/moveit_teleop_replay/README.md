# moveit_teleop_replay

Review a recorded episode: pick one, play it back, scrub it, and see which parts
were actually engaged. The arm does not move — playback drives a **preview
robot** drawn beside the live one.

```
episodes directory  ->  episode_player  ->  DisplayRobotState  ->  RViz ghost arm
                             ^  ReplayCommand service
                             |
                        ReplayPanel (moveit_teleop_rviz)
```

Included in the teleop RViz by default (`replay:=true` on the teleop launch), so
you can review the take you just recorded without stopping anything. Signal
flow: [moveit_teleop_bringup/doc/architecture.md](../moveit_teleop_bringup/doc/architecture.md).

---

## Nodes

### `episode_player`

| | |
|---|---|
| Publishes | `~/robot_state` (`moveit_msgs/DisplayRobotState`), `joint_states_topic` (`JointState`), `target_topic` (`PoseStamped`), `~/status` (`ReplayStatus`) |
| Subscribes | `robot_description_topic` (`std_msgs/String`, latched) |
| Serves | `~/command` (`ReplayCommand`) — load, play, pause, seek, step, speed, loop, refresh |

| Parameter | Default | |
|---|---|---|
| `episodes_dir` | `~/teleop_episodes` | the teleop launch fills this from `recording.yaml`'s `output_dir`, so the panel can never list a different directory than the recorder writes to |
| `joint_states_topic` / `target_topic` | `/replay/joint_states` / `/replay/target_pose` | kept off the live robot's topics |
| `rate_hz` | `0.0` | `0` replays at the episode's own recorded rate |
| `publish_rate` | `30.0` | UI refresh |
| `autoload_latest` | `true` | open the newest episode on startup |
| `robot_description_topic` | `/replay_robot_description` | slash-free name on purpose: MoveIt's RobotState display derives an internal node name from it, and a `/` fails the **entire** RViz config load |
| `preview_color` | `[0.15, 0.8, 0.35, 1.0]` | RGBA of the ghost arm |

The available-episode list rides on `~/status` rather than behind a service, so
a panel is correct the moment it connects and stays correct when a new episode
is recorded — no polling, no refresh button to forget.

### `description_publisher`

Latches a string onto a topic and keeps latching it. Two instances publish the
preview robot's URDF and SRDF: MoveIt's RobotState display builds its own
`RobotModel` and needs both, and nothing publishes a semantic description
outside a running `move_group`.

### `tf_prefixer`

Republishes `/replay/tf` and `/replay/tf_static` under a prefix.
`robot_state_publisher` in Jazzy has no `frame_prefix` parameter, and two
publishers of the *same* frame names interleave rather than merge — the tree
flickers between two robots. `root_frame` is attached to the live tree so the
preview appears in the same world.

---

## Launch

| Launch | Use |
|---|---|
| `replay_nodes.launch.py` | the machinery only, no RViz. What the teleop launch includes. |
| `replay.launch.py` | the same, plus a dedicated RViz and a `robot_state_publisher`, for reviewing episodes with nothing else running. |

Both **require** the recorded arm's model:

```bash
ros2 launch moveit_teleop_replay replay.launch.py \
    replay_urdf_xacro:=<path>.urdf.xacro \
    replay_srdf_xacro:=<path>.srdf.xacro \
    replay_xacro_args:="<the args the robot bringup uses>"
```

There is no default, deliberately. Nothing in a bag records which arm produced
it, so a wrong model shows a plausible-looking arm in the wrong pose with no
error anywhere. Those three values are exactly the robot config's
`files.preview_urdf` / `preview_srdf` / `preview_xacro_args`, which is what the
teleop launch passes — so prefer

```bash
ros2 launch moveit_teleop_bringup teleop.launch.py \
    robot_config_package:=<arm>_moveit_config replay:=true
```

and let the robot config supply them. Do not run both at once: the teleop launch
already puts the panel and the preview robot in the RViz it opened.

---

## Library

| | |
|---|---|
| `library.discover(directory)` | `EpisodeSummary` per episode, newest first |
| `library.mode_timeline(summary)` | the recorded mode transitions, for marking up a scrub bar |
| `library.Timeline` | resampled frames, through `moveit_teleop_record.reader.Episode` |
| `library.MODE_TELEOP` | which part of an episode was engaged |

## Tests

```bash
python3 -m pytest moveit_teleop_replay/test -q
```

Includes the guard that a Piper episode cannot be silently replayed onto a
Franka preview: the two joint-name sets must not intersect.
