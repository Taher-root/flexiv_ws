# moveit_teleop_bringup

Launch files and session config for the XR&nbsp;&rarr;&nbsp;MoveIt teleoperation stack.

A robot is selected by **one argument**. Everything robot-specific lives in that
robot's own MoveIt config package; no package in this repository names a link, a
joint, a frame, a controller or a vendor.

```bash
ros2 launch moveit_teleop_bringup teleop.launch.py \
    robot_config_package:=agx_arm_moveit
```

That reads `<share>/config/teleop.yaml` from the named package, and that file
names its own planning scene, RViz view and replay-preview xacros relative to
itself.

## Documentation

| Document | Read it when |
|---|---|
| [doc/bringup.md](doc/bringup.md) | You want to run it. Commands for sim, hardware, and the IQ9075 kit. |
| [doc/architecture.md](doc/architecture.md) | You want the signal-flow charts: what runs, what it publishes, and the order the guards sit in. |
| [doc/config-reference.md](doc/config-reference.md) | You need to know what a setting does, or what it defaults to. |
| [doc/tuning.md](doc/tuning.md) | The arm works but does not feel right. |
| [doc/new-arm.md](doc/new-arm.md) | You are bringing up an arm the stack has never seen. |
| [doc/diagnostics.md](doc/diagnostics.md) | Something is wrong and you want the shortest path to why. |
| [doc/datasets.md](doc/datasets.md) | You want training data: record, export to LeRobot, fine-tune GR00T or π0.5. |

Each package documents its own nodes, topics and parameters:
[core](../moveit_teleop_core/README.md) ·
[ik](../moveit_teleop_ik/README.md) ·
[devices](../moveit_teleop_devices/README.md) ·
[record](../moveit_teleop_record/README.md) ·
[replay](../moveit_teleop_replay/README.md) ·
[msgs](../moveit_teleop_msgs/README.md) ·
[rviz](../moveit_teleop_rviz/README.md)

## Layout

```
<arm>_moveit_config/config/
  teleop.yaml          <- robot_config_package:=<arm>_moveit_config reads this
  teleop_scene.yaml    <- named by files.scene, RELATIVE to teleop.yaml
  teleop.rviz          <- files.rviz          (optional)
  <arm>.urdf.xacro     <- files.preview_urdf  (optional)
  <arm>.srdf.xacro     <- files.preview_srdf  (optional)

moveit_teleop_bringup/config/
  recording.yaml       session-wide; owns the episodes directory
  servo.yaml           the DEPRECATED solver:=servo path
  teleop.rviz          fallback RViz view
  samples/
    template_teleop.yaml   <- start a new arm here
    template_scene.yaml
    piper_teleop.yaml  piper_scene.yaml  fr3_teleop.yaml
```

The two live configs are
`agx_arm_moveit/config/teleop.yaml` and
`franka_teleop_bringup/config/teleop.yaml`. The files under `config/samples/`
are reference copies plus an annotated template; nothing reads them.

## Config resolution

```
explicit launch argument   >   the robot's teleop.yaml   >   FALLBACKS in _teleop_config.py
```

A `DeclareLaunchArgument` default always beats a config file, because launch
cannot tell "the caller did not pass this" from "the caller passed exactly the
default". So every argument the config owns is declared with the sentinel
`UNSET = '<config>'`, and an `OpaqueFunction` positioned after all declarations
and before all nodes fills them in. Its position in the list is load-bearing.

**A key the resolver does not know is an error**, not a silent skip. If you add
a setting and the launch refuses it, the setting does not exist — look it up in
[doc/config-reference.md](doc/config-reference.md) rather than assuming it took
effect.

## Tests

```bash
python3 -m pytest moveit_teleop_bringup/test -q
```

They cover resolution precedence, fallback completeness, the unknown-key
refusal, package resolution, and — statically — that every config key is
actually declared as a launch argument and that no launch file reads a
configuration it never declares. Both of those last two caught real aborts.
