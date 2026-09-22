# aico2_moveit_config

MoveIt configuration for AICO2: two Rizon 4 arms on a 2-DoF waist.

Built by hand rather than by the MoveIt Setup Assistant, because the original
MoveIt packages this workspace refers to (`aico2_bringup`, `aico2_vr_teleop`)
are not here — see `docs/moveit_status.md`. If they turn up, prefer their SRDF
to this one: it will have the full collision matrix.

## How it executes

There is no `ros2_control` in this system. `aico2_left_arm_driver` /
`aico2_right_arm_driver` serve `FollowJointTrajectory` themselves and stream to
the Rizon controllers over RDK. So MoveIt uses
`MoveItSimpleControllerManager` with `moveit_manage_controllers: false`.

MoveIt builds the action name as `<controller_name>/<action_ns>`, so the
controller names in `config/moveit_controllers.yaml` **must** be `left_arm` and
`right_arm` to reach `/left_arm/follow_joint_trajectory` and
`/right_arm/follow_joint_trajectory`. Renaming them to the conventional
`*_controller` silently breaks execution: planning succeeds, execution reports
no controller found.

## Running it

MoveIt does not start the drivers; it plans for them.

```bash
ros2 launch flexiv_amr_bringup arms.launch.py          # drivers + RSP + joint states
ros2 launch aico2_moveit_config move_group.launch.py   # planning
ros2 launch aico2_moveit_config moveit_rviz.launch.py  # optional GUI
```

Before trusting MoveIt, confirm the layer underneath it works:

```bash
python3 ../aico2_left_arm_driver/scripts/check_arm_ros.py --send-goal --yes-move
```

That sends a `FollowJointTrajectory` goal directly. If it fails, MoveIt will
fail the same way with less explanation.

## Groups and states

| Group | Chain | Joints |
| ----- | ----- | ------ |
| `left_arm` | `Left_link0` → `Left_flange` | `Left_joint1..7` |
| `right_arm` | `Right_link0` → `Right_flange` | `Right_joint1..7` |
| `both_arms` | composite | all 14 |

Named states per arm: `home` (matches the drivers'
`mock_idle_joint_positions`) and `ready`. **Plan to `ready` before engaging
teleop** — `home` has `joint6 = 0`, a wrist singularity where MoveIt Servo
halts.

The waist (`AGV_Joint1/2`) is declared `passive`. It carries both arm bases via
fixed joints, and `aico2_waist_driver` publishes its real position, so MoveIt
plans around wherever the waist currently is. Nothing in ROS commands the waist
today; giving it a command path is a prerequisite for putting it in a planning
group.

## The collision matrix is partial

`config/aico2.srdf` is generated:

```bash
python3 scripts/generate_srdf.py \
    ../flexiv_amr_description/urdf/AICO2-Rizon4.urdf > config/aico2.srdf
```

66 pairs are disabled, derived from the kinematic tree: `Adjacent`
(parent/child across a joint) and `Welded` (rigidly connected through fixed
joints). The Setup Assistant's other two categories — `Never` (never collides
anywhere reachable) and `Default` (already colliding at zero) — need geometric
sampling against the meshes and are absent.

Consequences: planning works, but every state check tests pairs that can never
collide, so planning is slower than it needs to be, and self-collision checks
between the two arms and the AMR body are conservative — MoveIt may refuse
poses that are physically fine. Running the Setup Assistant once on this URDF
and merging its `disable_collisions` block fixes both.

## IK

KDL, because it ships with MoveIt. It is a numeric solver with no notion of
the arms' redundant 7th joint, so it can fail on poses a redundancy-aware
solver would reach. `config/kinematics.yaml` has a commented `pick_ik` block to
swap in if IK failures become the constraint.

## Compliant execution

The drivers can execute MoveIt trajectories compliantly — same trajectories,
same action, `NRT_JOINT_IMPEDANCE` instead of `NRT_JOINT_POSITION`:

```bash
ros2 param set /left_arm/left_arm_driver joint_stiffness_ratio 0.3
```

with `joint_control_mode: impedance` in the driver's YAML. MoveIt is unaware
and unaffected. See `docs/rdk_version_and_compliance.md`.
