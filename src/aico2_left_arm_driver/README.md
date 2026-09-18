# aico2_left_arm_driver

Lifecycle node for one Flexiv Rizon arm (mock or `flexivrdk` 1.9).

VR teleop, gripper, and driver modes are documented in
[`../../docs/VR_TELEOP.md`](../../docs/VR_TELEOP.md).

## Mock mode

```bash
ros2 launch aico2_bringup mock.launch.py
```

## Hardware

```bash
ros2 launch aico2_bringup hardware.launch.py
```

Requires `flexivrdk==1.9.0`, serial from Flexiv Elements, motion bar Auto Remote.
See [docs/PHASE0_HARDWARE.md](../../docs/PHASE0_HARDWARE.md).

## Interfaces (namespace `/left_arm`)

### State

| Topic | Type |
|-------|------|
| `joint_states` | `sensor_msgs/JointState` |
| `status` | `aico2_msgs/ArmStatus` |
| `tcp_pose` | `geometry_msgs/PoseStamped` |
| `external_wrench` | `geometry_msgs/WrenchStamped` |
| `fault` | `std_msgs/Bool` |
| `mode` | `std_msgs/String` |
| `gripper/width` | `std_msgs/Float32` (if `gripper_enabled`) |

### Teleop (mutually exclusive with `follow_joint_trajectory`)

| Name | Type | Backend |
|------|------|---------|
| `cartesian_twist_cmds` | `TwistStamped` | `teleop_backend: cartesian` |
| `servo_joint_command` | `JointTrajectory` | `teleop_backend: servo` |
| `gripper_command` | `Float32` | trigger 0..1 → force grasp |
| `set_teleop_mode` | `SetBool` | enable/disable VR streaming |

### Planning & primitives

| Name | Type |
|------|------|
| `follow_joint_trajectory` | `FollowJointTrajectory` action |
| `execute_primitive` | `aico2_msgs/ExecutePrimitive` |
| `clear_fault` | `Trigger` |

## Key params (`config/left_arm_hardware.yaml`)

| Param | Default | Notes |
|-------|---------|-------|
| `teleop_backend` | `cartesian` | `cartesian` or `servo` |
| `gripper_enabled` | `true` | Flexiv-GN01 via RDK |
| `cartesian_max_linear_vel` | `0.3` | m/s cap for Cartesian teleop |

Right arm uses the same node code via `aico2_right_arm_driver`.
