# Bringup

Two terminals, always. The first is the robot's own MoveIt bringup; the second
is teleop, and it names the robot with one argument.

---

## AgileX Piper — real hardware

### 1. The arm

```bash
source ~/Documents/agileX_teleoperation_ws/setup_env.sh

ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
    can_port:=can0 \
    arm_type:=piper \
    effector_type:=agx_gripper \
    tcp_offset:="[0.0, 0.0, 0.1425, 0.0, 0.0, 0.0]" \
    auto_control_gate:=false \
    follow:=true \
    use_rviz:=false
```

`follow:=true` **matters and is easy to miss.** Without it,
`robot_state_publisher`, `move_group` and RViz all subscribe to
`control/joint_states` — which is the *command bus*, not feedback: the driver
**subscribes** to that topic and sends it over CAN. TF then tracks the command,
so the retargeter's anchor, its leash and the collision checker all measure
against a pose the arm does not occupy. Measured on this arm, open-loop:
median 46 mm of tracking error, p95 115 mm, ~140 ms of phase.

`auto_control_gate:=false` hands the CAN gate to the mode manager, which is
what makes the deadman reach the hardware instead of only the software.

### 2. Teleop

```bash
ros2 launch moveit_teleop_bringup teleop.launch.py \
    robot_config_package:=agx_arm_moveit \
    joint_states_topic:=/feedback/joint_states \
    hardware_gate_enabled:=true \
    record:=true
```

`arm_type`, `effector_type` and `tcp_offset` are **not** teleop arguments. They
belong to the robot bringup above. Everything they used to feed comes from
`agx_arm_moveit/config/teleop.yaml`.

`record:=true` is the one people forget. Without it no episode is written and
the replay panel stays empty.

### Before the first run on a cold board

```bash
# CAN up (systemd does this at boot; this is the manual path)
sudo piper-can-up
ip -brief link show can0          # expect: can0  UP

# Arm must NOT be in drag-teach mode. Check after bringup:
ros2 topic echo /feedback/arm_status --once
#   ctrl_mode: 1   -> CAN instruction control, good
#   ctrl_mode: 2   -> TEACHING_MODE; the arm ignores CAN commands by design
```

`ctrl_mode: 2` is the cause of "homing timed out" that looks like every other
fault, so check it before anything else.

---

## AgileX Piper — simulation

```bash
ros2 launch agx_arm_moveit demo.launch.py \
    use_rviz:=false auto_control_gate:=false \
    effector_type:=agx_gripper \
    tcp_offset:="[0.0, 0.0, 0.1425, 0.0, 0.0, 0.0]"

ros2 launch moveit_teleop_bringup teleop.launch.py \
    robot_config_package:=agx_arm_moveit
```

Mock hardware echoes commands, so `control/joint_states` *is* the state and
`follow` is irrelevant here.

---

## Franka FR3 — simulation

```bash
ros2 launch franka_teleop_bringup demo.launch.py
ros2 launch moveit_teleop_bringup teleop.launch.py \
    robot_config_package:=franka_teleop_bringup
```

`franka_teleop_bringup/launch/teleop.launch.py` is an alias for exactly the
second command, so `ros2 launch franka_teleop_bringup teleop.launch.py` also
works and forwards every argument.

---

## No robot, no headset

```bash
ros2 launch moveit_teleop_bringup teleop.launch.py \
    robot_config_package:=agx_arm_moveit \
    device:=fake pattern:=circle radius:=0.35
```

Everything downstream of the device is identical — that is the point of the
device seam. The 0.35 m circle is the discriminator that used to freeze the
deprecated servo path.

Patterns: `circle`, `square`, `ramp`, `wobble`, `static`, and two that exist to
break things — `clutch_storm` (finds anchoring bugs) and `dropout` (finds
missing watchdog paths).

`plane:=xz` sweeps vertically and is the only way the scripted device can reach
a table, so it is what you use to exercise a virtual wall.

---

## The headset

`device` defaults to `xr`, so the teleop command above already starts the WebXR
bridge. The headset browses to `https://<this-host>:8443`.

WebXR only runs in a secure context, so a certificate is needed once per host:

```bash
bash $(ros2 pkg prefix moveit_teleop_devices)/share/moveit_teleop_devices/scripts/make_cert.sh <this-host-lan-ip>
```

If the page does not load, the usual cause is the host firewall. The node
prints the exact `ufw` rule to add, and warns only when ufw is genuinely
filtering rather than merely installed.

### Controls

| Control | Action |
|---|---|
| middle-finger trigger (squeeze) | clutch — level-triggered deadman |
| index trigger | gripper, analog |
| A / X | toggle recording |
| B / Y | discard last episode |
| thumbstick **click** | home to `ready_pose` |

Homing is refused while the clutch is held: it sweeps the arm across the
workspace, so it wants a button you cannot brush by accident while steering.

---

## On the IQ9075 dev kit

The Qualcomm Dragonwing IQ9075 EVK (the board reports itself as
`Qualcomm Technologies, Inc. Addons IQ 9075 EVK`). The workspace lives at `~/Documents/agileX_teleoperation_ws` and
`setup_env.sh` at its root sources ROS and the overlay. Three helper functions
are defined by `~/.bash_aliases`:

```bash
arm      # the robot bringup, hardware, with follow:=true
camera   # the gripper camera: Gemini 335L, RGB only, 1280x720 at 15 fps
teleop   # the teleop stack, pointed at real feedback, recording on
```

Run `arm` in one terminal and wait for the controllers to spawn, then `camera`
and `teleop` in two more. `camera` is optional for teleop itself, but the
recorder warns loudly if an episode starts without it.

`camera` runs the Orbbec node directly rather than
`gemini_330_series.launch.py`. That launch file forwards every argument as a
node parameter, so the driver applies depth auto-exposure priority, which this
camera's firmware rejects — and the rejection aborts device setup in a retry
loop.

All three are plain shell functions — run `type camera` to see the command one
expands to — and extra arguments pass straight through:

```bash
teleop device:=fake pattern:=circle    # no headset
teleop record:=false                   # do not record this run
camera -p color_fps:=30                # a different camera rate
```

Solve times on this board: median 2.77 ms, p99 4.3 ms, against an 11.1 ms
budget at 90 Hz. The desktop figure is 0.7 ms, so the board is ~4× slower and
still has ample headroom.

---

## RViz on another machine

The kit has no monitor, so run the stack there with its RViz off and open the
view on a desktop. Both machines must be on the same LAN with the same
`ROS_DOMAIN_ID` (0 unless you changed it).

```bash
# on the kit
arm                                  # terminal 1
camera                               # terminal 2 (optional)
teleop use_rviz:=false               # terminal 3

# on the desktop
rviz2 -d $(ros2 pkg prefix moveit_teleop_bringup)/share/moveit_teleop_bringup/config/teleop.rviz
```

The desktop needs `rviz2`, MoveIt's RViz plugins (`moveit_ros_visualization`),
`moveit_teleop_msgs` and `moveit_teleop_rviz` — but **none of the robot's own
packages**: the model arrives over DDS on `/robot_description`, and the planning
scene on `/monitored_planning_scene`. Teleop is still driven entirely from the
headset; the desktop is a viewer.

**Open the DDS ports or you will see topic names and no data.** Fast DDS uses
multicast 7400 plus a unicast port per participant (7410, 7412, …), and a
default-deny firewall lets discovery out but blocks the replies, so
`ros2 topic list` works while `ros2 topic echo` hangs forever:

```bash
sudo ufw allow from <your subnet>/24 to any port 7400:7600 proto udp
```

Then check that data actually flows, not just that the topic exists:

```bash
PYTHONUNBUFFERED=1 ros2 topic hz /mode_manager/status
```

`PYTHONUNBUFFERED=1` matters whenever you pipe `ros2 topic hz` anywhere: its
output is block-buffered, so a `timeout` that kills it discards everything it
measured and looks exactly like silence.

If topics appear and vanish, or two machines see different sets, check for a
second active interface — Wi-Fi and Ethernet on the same subnet make discovery
pick an address the other side cannot reach. Bring one down.

---

## What "up" looks like

Four lines from `ik_node` are worth reading every single time:

```
group='arm' chain 'base_link' -> 'tcp_link' (6 joints) controller='arm_controller'
nominal posture from SRDF named state 'ready_pose', condition number 121.8, ...
speed from MoveIt joint_limits.yaml x rate_limit_scale 0.5 -> [2.5 ...] rad/s
guards: joint limits from the model, jump limit 20 deg (retry x50), ...
```

1. The model resolved, and to the right tip link.
2. The bias posture is not a singularity (above 500 this is a WARN).
3. The speed ceiling actually in force.
4. Every guard, with anything disabled printed at WARN.

Read the fourth line before trusting any tuning measurement: a guard reported
as off (`retry x0`) while it is in fact firing invalidates the comparison.

Then `mode_manager` should walk `IDLE -> ALIGNED` once input is healthy, and
`ALIGNED -> TELEOP` when you squeeze the clutch.
