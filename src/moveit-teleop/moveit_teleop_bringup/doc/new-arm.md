# Adding an arm

If the arm has a MoveIt config and a `JointTrajectoryController`, this is a
config file and an SRDF addition. **No package in this repository should need
editing** — the FR3 port is the proof. If you find yourself typing a link name
into YAML, stop.

---

## 1. Give the SRDF a `ready_pose`

The stack homes to a named group state and regularises toward one. Most vendor
SRDFs name none, or name only all-zeros.

```xml
<group_state name="ready_pose" group="arm">
  <joint name="joint1" value="0.0"/>
  <joint name="joint2" value="0.36"/>
  ...
</group_state>
```

Pick a posture with the elbow genuinely bent and the wrist clear of alignment —
then verify it in step 4 rather than trusting it.

## 2. Copy the template into the arm's MoveIt config package

```bash
SHARE=$(ros2 pkg prefix moveit_teleop_bringup)/share/moveit_teleop_bringup

cp $SHARE/config/samples/template_teleop.yaml \
   src/my_arm_moveit_config/config/teleop.yaml
cp $SHARE/config/samples/template_scene.yaml \
   src/my_arm_moveit_config/config/teleop_scene.yaml
```

Make sure the package installs the directory:

```cmake
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
```

## 3. Fill in the six required values

Everything else can stay on its default for the first run.

```yaml
move_group:        # the SRDF planning group
ready_state:       # the group state from step 1
nominal_state:     # usually the same
controllers.arm:   # the ros2_control controller for those joints
retargeting.per_solver.ik.translation_scale:   # start at 1.0
ik.rate_limit_scale:                           # start at 0.2
```

## 4. Launch against the scripted device and read four lines

```bash
ros2 launch moveit_teleop_bringup teleop.launch.py \
    robot_config_package:=my_arm_moveit_config \
    device:=fake pattern:=circle
```

In `ik_node`'s output, confirm all four:

```
group='arm' chain 'base_link' -> 'tool0' (6 joints) controller='arm_controller'
   ^ the model was resolved. A wrong tip link means your SRDF chain is wrong.

nominal posture from SRDF named state 'ready_pose', condition number 8.2
   ^ above 500 this is a WARN and your ready_pose needs work.

speed from MoveIt joint_limits.yaml x rate_limit_scale 0.2 -> [...] rad/s
   ^ the ceiling actually in force. Sanity-check the numbers.

guards: joint limits from the model, jump limit 20 deg (retry x50),
        collision policy 'stop', 1 virtual wall(s)
   ^ anything disabled prints separately at WARN.
```

## 5. Give it an RViz view

Copy the fallback and change exactly two things:

- `Fixed Frame` → your URDF root link
- the TF display's `Filter (whitelist)` regex → `<your tip link>|teleop_target`

Then name it as `files.rviz`. Showing all frames instead makes the gap between
commanded and actual impossible to see, which is the entire point of the view.

## 6. Declare the table

Measure the real surface relative to the link you anchor to, leave a few
millimetres of gap below the mounting plane so the box does not intersect the
robot base, and size `wall_clearance` per
[tuning step 5](tuning.md#5-size-wall_clearance-against-tracking-error).

## 7. The robot model files — for replay and for datasets

Name `files.preview_urdf`, `files.preview_srdf` and `files.preview_xacro_args`.
They must build the **same** robot the bringup built, tool offset included.

Teleop runs without them (replay just loses its ghost arm), but **exporting
training datasets requires them**: the export runs offline, and derives the
dataset's joint layout and units from this model. Check it with one episode —
the first lines `convert` prints are the layout it derived:

```bash
ros2 run moveit_teleop_record convert --robot-config-package my_arm_moveit_config \
    ~/episodes/one_episode /tmp/check_dataset
```

See [moveit_teleop_record/README.md](../../moveit_teleop_record/README.md).

## 8. Then hardware

- Point `joint_states_topic` at the driver's real feedback, not the command echo.
- Enable `safety.hardware_gate` if the driver advertises one, and decide **what
  kind of gate it is** — this is the one hardware question the stack cannot
  answer for you:
  - a **motion enable** (nothing moves while it is shut) keeps the default
    `open_for_planning: true`;
  - a **streaming-mode select** needs `open_for_planning: false`, because a
    driver that offers one generally refuses to run its trajectory action while
    that mode is active. Read the driver's trajectory goal callback: if it
    rejects goals while streaming is on, you need `false`. Getting it wrong is
    silent — every planned home falls back to an unchecked interpolation while
    still reporting as planned.
- Check `home_via_moveit`. The default plans homing through `move_group`, which
  needs a controller manager in the arm's MoveIt config; set it `false` only
  for an arm whose MoveIt stack cannot execute trajectories.
- Re-check `wall_clearance` against the tracking error you now have.

---

## What you should never have to do

Write a launch file, edit any package in this repository, or name a link, joint,
frame, gripper joint value or joint limit in YAML.

All of those are read at runtime from `robot_description` and
`robot_description_semantic` on the running `/move_group`, which also guarantees
teleop reasons about exactly the robot the planner is using:

| What | Where it comes from |
|---|---|
| planning frame | the URDF root link |
| group base/tip link | the SRDF `<chain>` |
| joint names, in order | walking the URDF base → tip |
| joint position limits | URDF `<limit>` |
| joint velocity limits | MoveIt `joint_limits.yaml`, falling back to the URDF |
| end-effector link | SRDF `<end_effector parent_link=...>` |
| named postures | SRDF `<group_state>` |
| gripper open/closed values | SRDF group states matching `*open*` / `*close*` |
| which controller to publish to | move_group's `moveit_simple_controller_manager` parameters |

## The FR3 as a worked example

`franka_robot/franka_teleop_bringup/` contains only what a new robot
legitimately needs: a URDF/SRDF pair with mock `ros2_control`, a
`joint_limits.yaml`, an RViz view, a scene file, and a `teleop.yaml` that is
structurally identical to the Piper's with different numbers.

What differs, and why:

| Setting | Piper (6-DOF) | FR3 (7-DOF) | Why |
|---|---|---|---|
| `move_group` | `arm` | `fr3_arm` | SRDF group name |
| `rotation_scale` | 0.8 | 0.5 | wrist range |
| `weight_regularization` | 0.005 | 0.02 | 7-DOF has a real nullspace to shape |
| `rate_limit_scale` | 0.5 | 1.0 | FR3's URDF limits are already conservative |
| nominal condition no. | 121.8 | 8.2 | reported by the startup check |

**One lesson from that port:** invented numbers get caught by geometry. An
early `joint_limits.yaml` with plausible-looking velocity limits understated the
URDF's by up to 2.8× (joint6: 1.5 against 4.18 rad/s) and produced 77 mm of
vertical wander on a path that should have been flat. Use the URDF's numbers
verbatim.
