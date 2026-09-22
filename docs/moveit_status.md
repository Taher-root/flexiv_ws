# MoveIt status in this workspace

Short version: **there is no MoveIt planning stack in this repo.** Not broken —
absent. What exists is the driver-side half of a system whose planning half
lives in two packages that are not here.

## What is present

| Thing | Where | Works? |
| ----- | ----- | ------ |
| `FollowJointTrajectory` action server | `arm_driver_node.py:600`, at `/<ns>/follow_joint_trajectory` | yes — this is what MoveIt would drive |
| `servo_joint_command` subscriber (`JointTrajectory`) | `arm_driver_node.py:624`, under `teleop_backend: servo` | yes — the input a MoveIt Servo node would publish to |
| Four `*_moveit_servo.yaml` driver configs | `aico2_{left,right}_arm_driver/config/` | they are *driver* params, not servo params |
| URDF with both arms, waist, sensors | `flexiv_amr_description/urdf/AICO2-Rizon4.urdf` | yes, and it is MoveIt-ready (see below) |

## What is absent

No SRDF. No `kinematics.yaml`. No `joint_limits.yaml` for MoveIt. No
`move_group` launch file. No `moveit_controllers.yaml`. No moveit_config
package at all. Nothing in git history either — these were never committed
here.

The four servo YAMLs name their missing counterparts in their own header
comments:

```
# Pair with: aico2_bringup/mock_moveit_servo.launch.py
#            aico2_vr_teleop/teleop_moveit_servo.launch.py
```

Neither `aico2_bringup` nor `aico2_vr_teleop` is in `src/`. And
`left_arm.yaml` refers to "the SRDF 'ready' state (joint6=+pi/2)", so an SRDF
exists somewhere — on the VR teleop machine, most likely, alongside those two
packages.

**So the first question is not "why doesn't the MoveIt code work" but "where
is it".** It is probably on one of the other two machines (the VR teleop host,
or one of the non-git `flexiv_ws` checkouts). Finding it beats rewriting it:
it already encodes the collision pairs and named states this robot needs.

## The URDF is ready for it

Checked, and it does not need changes to support a moveit_config:

- Single root link `base_link`, one connected tree, 30 links.
- 16 revolute joints: `AGV_Joint1/2` (waist), `Left_joint1..7`, `Right_joint1..7`.
- Both arms hang off `AGV_Pitch` via fixed `Left_Joint0` / `Right_Joint0`, so
  the waist is in each arm's kinematic chain but not in an arm planning group —
  it moves the arm base, which MoveIt handles as long as `/joint_states`
  carries the waist (it does, since `aico2_waist_driver`).
- Tips are `Left_flange` / `Right_flange` via fixed `*_link7_to_flange`.
- Every revolute joint has `<limit lower= upper= effort= velocity=>`, e.g.
  `Left_joint1` ±2.8798 rad, effort 123, velocity 2.0944.

## Checking the half that does exist

`src/aico2_left_arm_driver/scripts/check_arm_ros.py` verifies the ROS arm
stack in the order things fail: node present in the expected namespace,
lifecycle `active`, `/joint_states` flowing with this arm's seven joints and
how many publishers are contributing, and the `follow_joint_trajectory` server
up. With `--send-goal --yes-move` it builds a two-point trajectory from the
live joint positions and sends it, then reports the error code and how far the
joint actually moved.

```
source /opt/ros/jazzy/setup.bash
python3 check_arm_ros.py                                   # checks only
python3 check_arm_ros.py --send-goal --yes-move             # moves joint 4 by 15°
python3 check_arm_ros.py --namespace right_arm
```

A pass is the prerequisite for MoveIt, not a substitute: `move_group`
ultimately just sends goals to that same action server. If this fails, MoveIt
would fail for the same reason and tell you less about why.

## If the original packages cannot be found

A moveit_config for this robot needs, roughly:

1. **SRDF** — planning groups (`left_arm`: `Left_joint1..7`, tip
   `Left_flange`; same for right), named states (including the `ready` pose
   with `joint6=+pi/2`, because `home` has `joint6=0`, a wrist singularity
   where Servo halts), and `disable_collisions` pairs for adjacent links plus
   the two arms against the waist and AMR body.
2. **kinematics.yaml** — 7-DOF redundant arms, so KDL's numeric solver is a
   poor fit; prefer `pick_ik` or a generated IKFast/TRAC-IK solver.
3. **moveit_controllers.yaml** — a `FollowJointTrajectory` controller entry per
   arm pointing at `/left_arm/follow_joint_trajectory` and
   `/right_arm/follow_joint_trajectory`. No `ros2_control` involved: this
   driver serves the action itself.
4. **move_group launch** — composed from `arms.launch.py` rather than starting
   its own drivers.

Two things to get right, given this robot:

- The waist (`AGV_Joint1/2`) is commanded by nothing in ROS today; the waist
  driver only *reads* it. Either keep it out of every planning group and let
  MoveIt treat it as a moving base, or add a command path first.
- The driver rejects a trajectory while teleop is active, and rejects one that
  omits any of the seven joint names (`reorder_to_driver`). A MoveIt group that
  plans for a subset of the arm's joints will be rejected, not partially
  executed.
