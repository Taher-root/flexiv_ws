# rizon10s_grippers

Flexiv RDK gripper helpers for Rizon 10s VR teleop.

## Force-grasp teleop

- **Front trigger** → proportional `Grasp(-force)` (closes until target force)
- **Trigger released** → `Move(open)`
- **Grip (squeeze)** → arm clutch only (unchanged)

Flow:

```
VR triggerValue  →  /arm/gripper_command (Float32)
                 →  arm_driver ForceGraspController
                 →  flexivrdk.Gripper.Grasp() / Move()
```

Arm teleop runs in parallel (Cartesian or Servo); gripper uses the same RDK
robot handle inside the arm driver.

## Params (on `arm_driver`)

| Param | Default | Meaning |
|-------|---------|---------|
| `gripper_enabled` | `true` | Subscribe to `gripper_command` |
| `gripper_name` | `Flexiv-GN01` | Flexiv Elements tool name |
| `gripper_max_force_fraction` | `0.75` | Scale trigger → max force |
| `gripper_open_threshold` | `0.08` | Below → open |
| `gripper_grasp_threshold` | `0.15` | Above → grasp |

## Topics

| Topic | Type | Notes |
|-------|------|-------|
| `/arm/gripper_command` | `std_msgs/Float32` | VR trigger 0..1 |
| `/arm/gripper/width` | `std_msgs/Float32` | Measured width (m) after commands |
