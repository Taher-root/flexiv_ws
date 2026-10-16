# moveit-teleop

VR teleoperation for any MoveIt-described arm, built for collecting
imitation-learning data. A Meta Quest controller drives the arm through a full
inverse-kinematics solver at 90 Hz; episodes record to MCAP and export to a
LeRobot v2.1 dataset that **GR00T** and **π0.5** fine-tune on.

**One argument selects the robot.**

```bash
ros2 launch moveit_teleop_bringup teleop.launch.py \
    robot_config_package:=<arm>_moveit_config
```

No file in this repository names a link, a joint, a frame, a controller or a
vendor. Everything robot-specific is either read at runtime from the running
`move_group` or declared in one file in the arm's own MoveIt config package. The
same code drives an AgileX Piper (6-DOF), a Franka FR3 (7-DOF) and a Flexiv
Rizon 10s (7-DOF) with no changes.

> Adding an arm is a config file, not a patch. There is a guide
> ([doc/new-arm.md](moveit_teleop_bringup/doc/new-arm.md)) that walks through it.

---

## Why full IK rather than differential IK

`moveit_servo` takes a Jacobian step, so a joint that reaches its limit simply
halts — the arm jams while the target runs away from it. Here the joint limits
and any virtual walls are **constraints in an optimisation solved every frame**,
so an unreachable target yields the closest *feasible* pose and the arm
reconfigures instead of stopping. Forward kinematics is an analytic CasADi
expression, so the Jacobian is exact automatic differentiation; a solve costs
~0.7 ms on a desktop and ~2.8 ms on an ARM dev board against an 11 ms budget.

## What you get

| | |
|---|---|
| Teleoperation | clutched relative-pose retargeting, operator-heading or tool reference frames, EMA smoothing, virtual walls the tool *slides* along |
| Safety | a real deadman (the solver is actively paused, not just starved of input), branch-flip refusal with a retry, velocity rate limiting from the robot's own limits, collision-aware posture moves, a hardware gate that follows the command path |
| Data | MCAP recording of any declared stream, aligned on capture time, exported as LeRobot v2.1 with `meta/modality.json` for GR00T — verified against GR00T's own loader and openpi's pinned lerobot |
| Review | an RViz panel that scrubs a recorded episode onto a ghost arm beside the live one |

## Install

ROS 2 Jazzy with MoveIt 2, and CasADi for the solver.

```bash
pip install casadi
mkdir -p ~/teleop_ws/src && cd ~/teleop_ws/src
git clone git@github.com:Aadith-Kumar/moveit-teleop.git
cd ~/teleop_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

You also need a MoveIt config package for your arm — the same one you would use
for planning. If you do not have one yet, MoveIt's Setup Assistant generates it
from your URDF.

Most of the suite runs with no ROS graph and no robot — 350 tests of 373:

```bash
for p in moveit_teleop_core moveit_teleop_ik moveit_teleop_devices \
         moveit_teleop_record moveit_teleop_replay moveit_teleop_bringup; do
    python3 -m pytest src/moveit-teleop/$p/test -q
done
```

The other 23 **skip, naming what they need**: they check the kinematics against
a real arm's URDF (forward kinematics against an independent oracle, the
analytic Jacobian against finite differences, convergence and the limit
constraints), so they run once any arm's MoveIt config package is installed
alongside. `21 skipped — Piper URDF not built` is the expected output on a
fresh clone, not a failure.

## Start here

| | |
|---|---|
| [doc/architecture.md](moveit_teleop_bringup/doc/architecture.md) | **what runs and how the signals flow** — read this first if you are changing anything |
| [doc/new-arm.md](moveit_teleop_bringup/doc/new-arm.md) | bringing up an arm the stack has never seen |
| [doc/bringup.md](moveit_teleop_bringup/doc/bringup.md) | commands for simulation, hardware and an ARM dev kit |
| [doc/config-reference.md](moveit_teleop_bringup/doc/config-reference.md) | every setting, what it does, what it defaults to |
| [doc/tuning.md](moveit_teleop_bringup/doc/tuning.md) | it works but does not feel right |
| [doc/diagnostics.md](moveit_teleop_bringup/doc/diagnostics.md) | something is wrong |
| [doc/datasets.md](moveit_teleop_bringup/doc/datasets.md) | record, export, fine-tune |

## Packages

| Package | |
|---|---|
| [moveit_teleop_msgs](moveit_teleop_msgs/README.md) | the interface: 6 messages, 2 services, no code |
| [moveit_teleop_core](moveit_teleop_core/README.md) | retargeting, mode arbitration and the deadman, gripper, scene |
| [moveit_teleop_ik](moveit_teleop_ik/README.md) | full IK as a constrained optimisation, every frame |
| [moveit_teleop_devices](moveit_teleop_devices/README.md) | WebXR controller driver, scripted device |
| [moveit_teleop_record](moveit_teleop_record/README.md) | episode recorder and the LeRobot exporter |
| [moveit_teleop_replay](moveit_teleop_replay/README.md) | scrub a recorded episode onto a preview arm |
| [moveit_teleop_rviz](moveit_teleop_rviz/README.md) | the RViz replay panel |
| [moveit_teleop_bringup](moveit_teleop_bringup/README.md) | launch files, config resolution, docs |

Only `moveit_teleop_bringup` knows the stack exists as a whole; anything lower
in the dependency graph is usable on its own. `moveit_teleop_ik` is a working IK
node for anything that publishes a `PoseStamped`, and `moveit_teleop_record`
exports datasets with no ROS graph running at all.

## Robot configs

A robot joins by adding `config/teleop.yaml` to its own MoveIt config package.
Annotated starting points are in
[`moveit_teleop_bringup/config/samples/`](moveit_teleop_bringup/config/samples/):

| | |
|---|---|
| `template_teleop.yaml` | start here — every setting, with the six that genuinely need a value marked REQUIRED |
| `template_scene.yaml` | obstacles and virtual walls |
| `piper_teleop.yaml`, `piper_scene.yaml` | a 6-DOF arm, tuned against hardware |
| `fr3_teleop.yaml` | a 7-DOF arm, for contrast — the posture term is nearly free where a nullspace exists |

## Status

Driven on real hardware (AgileX Piper) and in simulation (Franka FR3, Flexiv
Rizon 10s). Honest about its edges:

- **Acceleration is not enforced on the streaming path.** MoveIt's acceleration
  limits are planning limits; smoothing and the velocity ceiling bound the jerk.
  Planned posture moves are time-parameterised properly.
- **`solver:=servo` is deprecated**, kept only as an A/B reference.
- **openpi (π0.5) needs its own `TrainConfig`** and policy transform in openpi's
  source; the dataset loads with openpi's pinned lerobot, but that config is not
  written here.
- Virtual walls are enforced on the *command*, and an open-loop arm trails it.
  Close the loop (drive TF from measured feedback) before tightening clearances.

## Licence

Apache-2.0. See [LICENSE](LICENSE).
