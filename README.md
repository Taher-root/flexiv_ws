# AICO2 dual-arm mobile manipulator

ROS 2 Jazzy workspace for the Flexiv **AICO2-4**: two 7-DoF Rizon4 arms on a
2-DoF waist, mounted on an FMR 300 differential-drive chassis, with an
Nvidia Jetson as the onboard computer.

This README covers setup, how to bring the system up one piece at a time, and
the failure modes that cost the most time to diagnose. Read
[§2 Safety constraints](#2-safety-constraints-the-software-does-not-enforce)
before moving anything.

---

## Contents

1. [Hardware reference](#1-hardware-reference)
2. [Safety constraints the software does not enforce](#2-safety-constraints-the-software-does-not-enforce)
3. [Setup](#3-setup)
4. [Building](#4-building)
5. [Running, one layer at a time](#5-running-one-layer-at-a-time)
6. [Running the whole system](#6-running-the-whole-system)
7. [Multi-machine: RViz on a laptop](#7-multi-machine-rviz-on-a-laptop)
8. [Debugging playbook](#8-debugging-playbook)
9. [Known issues](#9-known-issues)

---

## 1. Hardware reference

### Network

Factory-configured, all on `192.168.1.0/24`. Set your laptop to a static
address in this subnet.

| Device | IP |
|---|---|
| Left arm | `192.168.1.100` |
| Right arm | `192.168.1.101` |
| FMR chassis | `192.168.1.110` |
| Edge AI computer (Jetson) | `192.168.1.103` |
| Wi-Fi module (LAN) | `192.168.1.254` |

Arm serial numbers, as configured in `flexiv_amr_bringup/launch/arms.launch.py`:

```python
LEFT_SN  = "Rizon4-063352"
RIGHT_SN = "Rizon4R-062077"
```

**The waist is wired through the left arm.** The right arm's waist-joint data
is transmitted via the left arm's communication module. If the left arm's IP
changes, you must also update `ip_address` in the right arm's
`programs/user_data/settings/dualRobotCommCfg.xml`. This is also why the waist
axes are owned by the left arm driver in this workspace, not by a separate node.

### Operating panel buttons

| Button | Function |
|---|---|
| Power on/off | Hold until the indicator stays **green** to power on. Hold **3 s** to power off. |
| **Reset** | **Clears chassis error messages.** This is the chassis equivalent of clear-fault. |
| E-Stop ×3 | Whole-machine emergency stop — halts both arms *and* the chassis. |
| Circuit breaker | Cuts device power entirely. |

Each arm's motion bar also has its own E-stop that halts only that arm.

### Chassis light strip

The chassis light strip is the **only** indication of chassis state that is
reachable without the vendor SDK. Learn these:

| LED | Meaning |
|---|---|
| **Slow blinking red** | **E-stop triggered, or an error has occurred** |
| Rapid flashing pinkish-purple | Chassis is blocked |
| Slow blinking blue | Moving straight |
| Slow blinking orange-yellow | Charging |
| Rapid flashing orange-yellow | Low battery |
| Green → dark red | Stationary; colour encodes battery level |
| Rainbow | Battery anomaly (unconfigured type, or BMS comms error) |

Error alarms have the highest display priority and supersede all lower states.
Updates roughly every 300 ms.

### Arm motion bar indicator

| Light | Meaning |
|---|---|
| Yellow | Power off |
| Green | In operation |
| White | Program stopped / paused |
| **Red** | **Fault** |
| Blue | Freedrive |

### Chassis onboard obstacle avoidance

The chassis decelerates and stops on its own, in firmware, independently of
anything Nav2 does. Factory zones:

| Zone | Default distance | Action |
|---|---|---|
| Red (emergency stop) | 0.2 m | Emergency decelerate at 2 m/s² to a stop |
| Yellow (stop) | 1 m | Navigation decelerate at 1 m/s² to a stop |
| Blue (deceleration) | 3 m | Decelerate to 0.2 m/s |

All zones extend 0.1 m beyond the robot's width on each side. The rear zone is
only active while reversing. Factory prediction distances: deceleration 3 m,
collision 1 m, emergency stop 0.2 m.

**This matters for debugging.** In a tight space the chassis will cap itself at
0.2 m/s or refuse to move, with `/cmd_vel` perfectly valid and Nav2 reporting
nothing wrong. Two independent obstacle systems are active at once: the
chassis firmware above, and Nav2's `collision_monitor`.

### Specifications (AICO2-4)

| | |
|---|---|
| Dimensions (stowed) | 900 × 800 × 1585 mm |
| Minimum aisle width | 900 mm |
| Weight | 281 kg |
| Chassis max speed | 1.0 m/s (Nav2 here is capped at 0.2 m/s) |
| Passability | ≤5 % slope / 5 mm step / 30 mm gap |
| Arm payload / reach | 4 kg / 780 mm per arm |
| Arm DoF | 7 per arm |
| Waist DoF / range | 2 — A1 ±90°, A2 0–90° |
| Control interface | RDK, 1 kHz |
| Battery | 48 V / 68 Ah, ~6 h, 2 h charge |

Battery thresholds: below 20 % triggers audible and light alarms; below 10 %
cuts power to all devices after 2 minutes.

---

## 2. Safety constraints the software does not enforce

**Nothing in this workspace checks any of the following.** They are
operational rules from the AICO2 manual, and violating them is a hardware
risk, not a software error.

1. **Do not move the arms or waist while the chassis is driving.** When the
   AICO2 is in motion the machine must be in the stowed position; movement of
   the arms and waist joint is prohibited.

   **Nothing stops you, and this was tested.** With a left-arm trajectory
   executing, the chassis was driven from keyboard teleop at the same time.
   The arm goal ran to `SUCCESS` and the chassis moved. Neither side refuses
   the other, and no alarm was raised. Nothing in this workspace couples them
   either: no node reads arm state before publishing `/cmd_vel`, and none
   reads chassis state before accepting a trajectory.

   So the prohibition is **procedural, and the operator is the only thing
   enforcing it.** Drive with the arms stowed.

   Test conditions, so nobody reads more into it than it shows: wrist roll
   only (`Left_joint7`, 20°) on an otherwise stowed arm, chassis rotating in
   place at reduced teleop speed. That establishes the combination is not
   blocked — not that any combination is safe. The stability and coasting
   limits in 3 and 4 below are why stowed still matters.

2. **Keep the chassis motor enabled whenever the arms or waist are moving.**
   The chassis motor has no brake, so it must stay enabled to hold position
   while the upper body moves.

3. **The chassis has no brake, so it coasts after an E-stop.** From maximum
   speed in the stowed position it travels up to **1.2 m** on inertia. Keep
   that clearance.

4. **Waist envelope limit.** Waist joint X2 must not exceed **30°** while X1 is
   between 30°–60° or −60°–−30°. Outside that, the chassis suspension makes the
   machine unstable when bending forward on one side. In this workspace X1 is
   `AGV_Joint1` (yaw) and X2 is `AGV_Joint2` (pitch).

5. **Auto (Remote) mode is required.** The arm motion bar must be in
   Auto (Remote) *before* power-on for RDK control to work. In Manual mode the
   arm is capped at 250 mm/s TCP and only moves while the enabling button is
   held, so ROS goals will appear to do nothing.

### Measured waist limits

The manual says A2 is 0–90°. Measured on `Rizon4-063352`:

| Joint | Range |
|---|---|
| `AGV_Joint1` (yaw) | −87.45° … +87.45° |
| `AGV_Joint2` (pitch) | **+2.50°** … +87.45° |

The pitch minimum is **not zero**. Commanding 0.0 is below the mechanical
limit. Positive pitch moves the torso **down**.

---

## 3. Setup

### Prerequisites

ROS 2 **Jazzy** (`ros-jazzy-desktop`), plus two vendor Python SDKs that are
**not rosdep keys** and must be installed by hand.

| Module | Needed by | Notes |
|---|---|---|
| `flexivrdk` | arm drivers, waist driver, `rdk_*.py` scripts | RDK **1.9.0**, matched to controller software v3.11. **2.x does not support Rizon.** |
| `flexivamr` | `status_monitor`, `velocity_controller` | AMR vendor SDK. **Not required** — see below. The chassis is driven over raw Seer TCP, not through this SDK |

Both import lazily, so the workspace builds without them.

**A missing `flexivrdk` is loud:** the arm driver's `on_configure` returns
FAILURE and the lifecycle node refuses to activate.

**`flexivamr` is not used by the running system at all.** The chassis is
driven by hand-rolled TCP against the Seer/Robokit API, not through the SDK:

| Node | Seer API | Port | Needs SDK? |
|---|---|---|---|
| `robokit_velocity_controller` | 2010 (motion) | 19205 | no |
| `odometry_publisher` | 1005 (encoder speed) | 19204 | no |
| `seer_lidar_publisher` | 1009 (laser point cloud) | 19204 | no |
| `seer_imu_publisher` | 1014 (IMU) | 19204 | no |
| `velocity_controller` | — | — | **yes**, and only runs with `use_robokit:=false` |
| `status_monitor` | — | — | **yes** |

With `use_robokit:=true` (the default), `velocity_controller` never launches.
That leaves `status_monitor` as the SDK's only consumer, and all it provides is
`/amr/status`, `/amr/emergency` and `/amr/blocked` — **which nothing in this
workspace subscribes to.** So on a machine without the SDK you lose chassis
status reporting and nothing else.

**Why it is done this way.** The `flexivamr` SDK does not support continuous
velocity control, at least in the versions available when this was built. Nav2
needs a steady stream of `/cmd_vel` at the controller rate, so the raw-TCP path
was written to send API 2010 directly at 20 Hz. That is the reason
`use_robokit` defaults to `true` and the reason the API numbers above are
hard-coded in `robokit_protocol.py` rather than called through a vendor
wrapper.

Treat `velocity_controller` as **superseded**, not as an equally valid
alternative. It is kept because it is the only code that holds the
gain-control handshake, which is useful reference if that ever turns out to
matter — but it cannot drive Nav2, and selecting it on a machine without the
SDK silently stops the chassis from moving at all.

**But its absence is silent, and that has cost real debugging time.**
`status_monitor` and `velocity_controller` both catch the `ImportError`, log it
**once** at startup, and then run forever as no-ops. The nodes appear in
`ros2 node list` and their topics appear in `ros2 topic list` — they simply
never publish. The practical cost is that a chassis E-stop becomes
unobservable from ROS, so **read the physical light strip instead** (see
[§1](#chassis-light-strip)). Check it explicitly:

**Installing them.** `flexivrdk` is distributed as a PyPI wheel. Pin the
version — see [`docs/rdk_version_and_compliance.md`](docs/rdk_version_and_compliance.md)
for why 2.x is the wrong SDK for this robot, not a newer one:

```bash
python3 -m pip install "flexivrdk==1.9.0"
```

It must be installed for the **same Python that ROS uses** — python3.12 on
Jazzy. Installing into a virtualenv ROS does not see is a common way to get an
`ImportError` from a node while `python3 -c "import flexivrdk"` succeeds in
your shell.

`flexivamr` is a vendor SDK, its distribution channel is not recorded in this
repo, and the default configuration does not need it. Obtain it from Flexiv if
you want chassis status reporting. Do not guess at a package name.

**Verify both, against the interpreter ROS will use:**

```bash
python3 -c "import flexivrdk; print('rdk', flexivrdk.__version__ if hasattr(flexivrdk,'__version__') else 'ok')"
python3 -c "import flexivamr; print('amr ok')"
ros2 topic hz /amr/status          # silence here means status_monitor is dead
```

**On a machine without `flexivamr`, `use_robokit` must stay `true`.** With
`false`, `velocity_controller` starts, fails to connect, never seizes control,
and silently discards every `/cmd_vel`. The chassis will not move and nothing
will say why.

A machine that only *views* (RViz, MoveIt panel) needs neither SDK.

### Get the code and resolve dependencies

```bash
git clone <repo> ~/flexiv_ws
cd ~/flexiv_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

`rosdep` reads every `package.xml`, so anything not declared there will not be
installed. If a node dies with `ModuleNotFoundError` or a launch file fails on
`PackageNotFoundError`, the dependency is missing from the relevant
`package.xml` — fix it there rather than apt-installing by hand, or the next
machine breaks the same way.

---

## 4. Building

```bash
cd ~/flexiv_ws
colcon build --symlink-install
source install/setup.bash
```

### colcon notes that matter for debugging

**Source `install/setup.bash` in every terminal, after every build.** A shell
sourced before a build will not see newly installed launch files, configs, or
entry points. Most "my change did nothing" reports are this.

**Data files are copied at build time.** `flexiv_amr_nav2/CMakeLists.txt` has:

```cmake
install(DIRECTORY config launch maps DESTINATION share/${PROJECT_NAME})
```

So a map or `.db` you drop into `src/flexiv_amr_nav2/maps/` after the last
build **is not** in the install share, and launch files resolve from the share.
Rebuild that package, or check both paths:

```bash
ls ~/flexiv_ws/src/flexiv_amr_nav2/maps/
ls $(ros2 pkg prefix flexiv_amr_nav2)/share/flexiv_amr_nav2/maps/
```

**Build one package instead of all thirteen:**

```bash
colcon build --packages-select flexiv_amr_nav2
colcon build --packages-up-to flexiv_amr_bringup    # that package and its deps
```

**See the real compiler or CMake output**, instead of colcon's summary:

```bash
colcon build --packages-select <pkg> --event-handlers console_direct+
```

**"Unable to order packages topologically"** means a circular dependency
between two packages' `package.xml` files. colcon refuses to build *anything*,
not just the cycle. The error names the packages involved. Inspect with:

```bash
colcon list --topological-order
colcon graph
```

**A stale `build/` or `install/` tree** causes errors that survive a code fix —
renamed targets, deleted files still installed, CMake cache pointing at old
paths. When something makes no sense:

```bash
rm -rf build install log && colcon build --symlink-install
```

`--cmake-clean-cache` is the lighter version for CMake-only staleness.

---

## 5. Running, one layer at a time

Each layer is independently launchable. Build up from the bottom when
something is broken — it is much faster than debugging the full stack.

Every terminal needs:

```bash
cd ~/flexiv_ws && source install/setup.bash
```

### 5.1 Chassis only — just make the robot move

The smallest useful test. Brings up the velocity controller, odometry, and
status monitor, and nothing else.

```bash
ros2 launch flexiv_amr_driver amr_driver.launch.py use_robokit:=true
```

> **`use_robokit` defaults to `false` in this launch file but `true` in
> `full_system.launch.py`.** Pass it explicitly. `true` selects
> `robokit_velocity_controller` (raw TCP to port 19205, no vendor SDK);
> `false` selects `velocity_controller`, which needs `flexivamr`.

Drive it from the keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Press **`z` several times first** — it starts at 0.5 m/s, which is too fast
indoors. Each press drops the speed 10 %.

> **`z` is not a brake.** This is standard `teleop_twist_keyboard`
> behaviour, not something this workspace adds. Its speed keys
> (`q` `z` `w` `x` `e` `c`) scale the speed but **do not clear the direction
> vector**, and the node publishes at the end of the same loop either way. So
> `z` means *"same direction, 10 % slower"* — it is a motion command. Press it
> while rolling and you keep rolling; press it after the robot has stopped and
> it **starts moving again** in the last direction.
>
> | To do this | Press |
> |---|---|
> | **Stop** | **`k`**, or any unbound key — the `else` branch publishes all zeros |
> | Slow down | `z`, but expect to move as well |
> | Set a safe speed | `z` repeatedly **before** the first direction key |
>
> In a narrow aisle: set the speed first, then steer, and stop with `k`.
> Confirm the behaviour on your own install with `ros2 topic echo /cmd_vel`
> while pressing `i` then `z` — you should see the same sign at a lower
> magnitude.

Each tap moves the chassis for at most 0.5 s: the node publishes once per
keypress and then blocks for the next one, and `robokit_velocity_controller`
sends a stop when no `/cmd_vel` has arrived within `cmd_timeout` (0.5 s). At
0.3 m/s that is ≤15 cm per tap, which makes it the right tool in tight
spaces — tap-to-move with a built-in deadman.

Verify:

```bash
ros2 topic echo /cmd_vel --once     # what the chassis is being told
ros2 topic echo /odom_raw --once    # what the chassis actually did
```

`/odom_raw` is the ground truth for motion. Do **not** use
`/amr/actual_velocity` — two nodes publish to it and one of them echoes the
*command* back as if measured.

### 5.2 Sensors

```bash
ros2 launch flexiv_amr_sensors sensors.launch.py
```

| Argument | Default | Meaning |
|---|---|---|
| `use_camera` | `true` | Bottom D456 + depth-to-laserscan |
| `use_top_camera` | `false` | Head-mounted D456; adds `/scan/top` |
| `use_visual_odom` | `true` | RTAB-Map visual odometry |
| `laserscan_topics` | `/scan/nav /scan/avoid /scan/depth` | Scans fed to the merger |
| `camera_serial` | `_333422304124` | Bottom camera |
| `camera_top_serial` | `_324422301136` | Top camera |

> **The two serial defaults are specific to our cameras and will not match
> yours.** A wrong serial means the RealSense node finds no device and
> `/scan/depth` never publishes, which then starves `/scan/merged`. Find yours
> with `rs-enumerate-devices -s`, and note the leading underscore the launch
> file expects:
>
> ```bash
> ros2 launch flexiv_amr_sensors sensors.launch.py camera_serial:=_<your serial>
> ```

Topics: `/scan/nav` and `/scan/avoid` (chassis lidars), `/scan/depth`
(depth camera), merged into **`/scan/merged`**.

> **The merger waits on every topic it is given.** Listing a scan that never
> publishes costs you `/scan/merged` entirely — and with it the costmaps' laser
> source, AMCL's input, and the collision monitor's. If you enable the top
> camera, `/scan/top` must be in `laserscan_topics`; if you disable it, it must
> not be.

```bash
ros2 topic hz /scan/merged    # expect ~30 Hz
ros2 topic hz /scan/nav       # expect ~15 Hz
ros2 topic hz /scan/avoid     # expect ~15 Hz
```

### 5.3 Mapping

Two independent backends. Use one.

```bash
# SLAM Toolbox — 2D laser SLAM
ros2 launch flexiv_amr_nav2 slam.launch.py

# RTAB-Map — lidar + visual, builds the .db used for localization
ros2 launch flexiv_amr_nav2 rtabmap_mapping.launch.py \
    rtabmap_db:=/path/to/output.db delete_db_on_start:=true
```

Save a SLAM Toolbox map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/flexiv_ws/src/flexiv_amr_nav2/maps/mymap
```

Rebuild `flexiv_amr_nav2` afterwards so the new map reaches the install share.

### 5.4 Localization

Exactly one of these may run — both publish `map → odom`, and running two
means a fight over the same transform.

```bash
# RTAB-Map against a prebuilt database (lidar + visual features)
ros2 launch flexiv_amr_nav2 rtabmap_localization.launch.py \
    rtabmap_db:=/path/to/rtabmap_backup.db

# AMCL against a pgm/yaml map (laser only)
ros2 launch flexiv_amr_nav2 localization.launch.py \
    map:=/path/to/map.yaml
```

Verify:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

A repeating translation and rotation means localization is live. Early
`Invalid frame ID "map"` lines are the startup race and resolve on their own.
Note the x, y, and **radian** yaw — you need them to compose a goal.

### 5.5 Nav2

```bash
ros2 launch flexiv_amr_nav2 navigation.launch.py
```

**Always verify the lifecycle chain before sending a goal:**

```bash
for n in controller_server smoother_server planner_server behavior_server \
         velocity_smoother collision_monitor bt_navigator waypoint_follower docking_server; do
  printf '%-22s ' "$n"; ros2 lifecycle get /$n 2>&1 | head -1
done
```

All nine must read `active [3]`. Reading the output:

| Result | Meaning |
|---|---|
| `active [3]` ×9 | Good |
| `Node not found` | Nav2 was never launched — check your launch arguments |
| `inactive [2]` | Configured but not activated: a node later in the list failed to configure, and `lifecycle_manager` aborted `startup()` |
| `unconfigured [1]` | **That** node is the blocker |

The manager configures in list order and aborts on the first failure, so the
`unconfigured [1]` entry names the culprit and everything before it is
stranded at `inactive [2]`. `bt_navigator` creates its action server in
`on_configure` but only accepts goals once *activated* — so an inactive chain
gives you a discoverable `/navigate_to_pose` that rejects every goal with
"Goal was rejected."

To activate by hand if you need to move on without fixing the root cause
(`bt_navigator` last, so its dependencies are up first):

```bash
for n in controller_server smoother_server planner_server behavior_server \
         velocity_smoother collision_monitor waypoint_follower bt_navigator; do
  ros2 lifecycle set /$n activate
done
```

### 5.6 Sending a navigation goal

Read your pose with `tf2_echo` (§5.4), then place the goal **more than 0.35 m
away**, because `xy_goal_tolerance` is 0.30 m and anything closer is declared
reached without the robot moving.

```
GOAL_X = x + D * cos(yaw)
GOAL_Y = y + D * sin(yaw)
```

With `D = 0.8` and the quaternion left at its current value so the robot
translates instead of turning first:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: GOAL_X, y: GOAL_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: QZ, w: QW}}}}" \
--feedback
```

Substitute real numbers — the literal placeholders fail with
`could not convert string to float`.

**Plan without moving**, to check a route is clear:

```bash
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
"{goal: {header: {frame_id: map}, pose: {position: {x: GOAL_X, y: GOAL_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: QZ, w: QW}}}, use_start: false, planner_id: 'GridBased'}"
```

`planner_server` has no connection to `/cmd_vel`, so this cannot move the
robot. But a successful plan is **not** proof the goal is clear: the planner
runs with `tolerance: 1.0` (it may return a path ending a full metre short) and
`allow_unknown: true` (it will route through unmapped cells).

**Judging the result — all four:**

| Check | Want |
|---|---|
| `number_of_recoveries` | stays `0` |
| `distance_remaining` | decreases monotonically |
| `navigation_time` | **> 2 s** — under ~100 ms means the tolerance fired and it never drove |
| final status | `SUCCEEDED` with `error_code: 0` |

Ctrl-C sends a cancel to the action server, which stops the controller and
zeros `/cmd_vel`. That is the software stop button.

#### Relevant tuning values

| Parameter | Value | Consequence |
|---|---|---|
| `xy_goal_tolerance` | 0.30 m | Goals closer than this succeed without moving |
| `xy_goal_tolerance` (DWB) | 0.15 m | A **second, different** value inside the `FollowPath` block. Goal success is decided by `general_goal_checker`'s 0.30, not this one — easy to misread |
| `yaw_goal_tolerance` | 0.5 rad | ~29° |
| `max_vel_x` | 0.2 m/s | |
| `max_vel_theta` | 0.5 rad/s | |
| **`min_vel_x`** | **0.0** | **DWB cannot reverse.** A goal behind the robot becomes spin-drive-spin |
| `robot_radius` | 0.20 m | |
| `PolygonStop.radius` | 0.30 m | Stop zone is 10 cm *outside* the robot's own footprint |
| `required_movement_radius` | 0.5 m / 10 s | Progress checker fails a stalled robot after ~10 s, then recovers |

To back up, either use teleop, or give a goal behind the robot with the
heading rotated 180° so it spins once and drives forward.

### 5.7 Arms

```bash
ros2 launch flexiv_amr_bringup arms.launch.py
```

| Argument | Default | Meaning |
|---|---|---|
| `mock_hardware` | `false` | `true` runs without a real robot |
| `joint_control_mode` | `position` | `position` = NRT_JOINT_POSITION, `impedance` = NRT_JOINT_IMPEDANCE (compliant) |
| `joint_stiffness_ratio` | `1.0` | Fraction of `K_q_nom` in impedance mode |
| `max_contact_torque` | `0.0` | Nm ceiling per arm axis against the environment; `0` leaves it unbounded |
| `control_waist` | `false` | `true` lets the left arm driver command the waist |
| `enable_waist_driver` | `false` | Publishes real `AGV_Joint1/2`; races `joint_state_merger`'s zeros |

Both drivers are **lifecycle nodes** and must reach `active`:

```bash
ros2 lifecycle get /left_arm/left_arm_driver
ros2 lifecycle get /right_arm/right_arm_driver
```

Check the ROS side without moving anything:

```bash
python3 src/aico2_left_arm_driver/scripts/check_arm_ros.py
```

Clear an arm fault:

```bash
ros2 topic echo --once /left_arm/fault
ros2 service call /left_arm/clear_fault std_srvs/srv/Trigger
ros2 service call /right_arm/clear_fault std_srvs/srv/Trigger
```

The driver also calls `clear_fault()` on activation, so relaunching clears a
minor fault. If `ClearFault()` keeps failing, check: E-stop released, motion
bar in Auto (Remote), and the arm not parked past a joint limit. A red light on
the motion bar means fault; if Elements shows no fault-clearing notification,
the Control Box needs a manual reboot.

#### Compliant (impedance) mode

```bash
ros2 launch flexiv_amr_bringup arms.launch.py \
    joint_control_mode:=impedance joint_stiffness_ratio:=0.3 \
    max_contact_torque:=15.0
```

`max_contact_torque` is clamped per axis against the robot's own `tau_max`, so
a value above the hardware limit is reduced rather than rejected.

### 5.8 MoveIt

```bash
ros2 launch aico2_moveit_config move_group.launch.py
```

With RViz:

```bash
ros2 launch aico2_moveit_config moveit_rviz.launch.py
```

Planning groups:

| Group | Use |
|---|---|
| `left_arm` | 7 DoF |
| `right_arm` | 7 DoF |
| `left_arm_waist` | Left arm + both waist axes, planned together |
| `both_arms` | **Avoid.** 14 DoF with no IK solver in `kinematics.yaml` — no interactive marker, and planning took ~123 s |

Command-line goals, without RViz:

```bash
# list the SRDF named states for a group
python3 src/aico2_moveit_config/scripts/moveit_goal.py --group left_arm --list

# plan only, no motion
python3 src/aico2_moveit_config/scripts/moveit_goal.py --group left_arm --joint 4 --degrees 10 --plan-only

# execute (--yes-move is required)
python3 src/aico2_moveit_config/scripts/moveit_goal.py --group left_arm --joint 4 --degrees 10 --yes-move

# move to a named state
python3 src/aico2_moveit_config/scripts/moveit_goal.py --group left_arm --named ready --yes-move

# random target within 25 deg of the current pose, reproducible with --seed
python3 src/aico2_moveit_config/scripts/moveit_goal.py --group left_arm --random --seed 42 --plan-only
```

Useful options: `--velocity-scaling` / `--acceleration-scaling` (default 0.2),
`--planning-time`, `--tolerance`, `--random-range`, `--limit-margin`.

Named states `home` and `ready` are defined for `left_arm` and `right_arm`
only — `both_arms` and `left_arm_waist` have none. `--joint` and `--random`
also accept only the two 7-DoF arm groups, since both index joints 1-7.

Validate the generated config against the URDF without a robot:

```bash
python3 src/aico2_moveit_config/scripts/check_config.py
```

### 5.9 Waist

The arm and waist share one RDK connection: the robot is **9 DoF**
(2 waist + 7 arm), and `SendJointPosition` takes the **full 9-element vector**
with `q[0:2]` as the external waist axes.

Through ROS, via the left arm driver:

```bash
ros2 launch flexiv_amr_bringup arms.launch.py control_waist:=true
```

`/joint_states` then carries `AGV_Joint1` and `AGV_Joint2`, and the left arm's
`follow_joint_trajectory` accepts 9-joint goals. Plan arm and waist together
with the `left_arm_waist` MoveIt group.

Standalone, bypassing ROS entirely:

```bash
python3 src/aico2_left_arm_driver/scripts/rdk_waist_move.py Rizon4-063352 \
    --axis pitch --degrees 5 --yes-move
```

`--axis` is `yaw` or `pitch`. **Positive pitch moves the torso down.** Respect
the measured limits in [§2](#measured-waist-limits) — pitch cannot go below
+2.50°, and the envelope rule couples pitch to yaw.

> `LockExternalAxes(False)` must be called while the robot is **IDLE**, before
> `SwitchMode`. The driver does this on the trajectory path; if you write your
> own RDK script, the order matters.

---

## 6. Running the whole system

```bash
ros2 launch flexiv_amr_bringup full_system.launch.py
```

**The defaults are mapping-only: `use_slam:=true`, `use_nav:=false`.** The bare
command gives you SLAM Toolbox and *no Nav2*, which presents as every Nav2
lifecycle node reporting "Node not found".

| Goal | Command |
|---|---|
| Mapping with SLAM Toolbox | `full_system.launch.py` (default) |
| Mapping with RTAB-Map | `full_system.launch.py use_slam:=false use_rtabmap:=true` |
| **Localization + Nav2** | `full_system.launch.py use_slam:=false use_nav:=true` |
| Nav2 against a pgm map | `full_system.launch.py use_slam:=false use_nav:=true localization_backend:=amcl map:=/path/map.yaml` |
| Add MoveIt | append `use_moveit:=true` |

Full argument list:

| Argument | Default |
|---|---|
| `use_camera` | `true` |
| `use_top_camera` | `false` |
| `use_visual_odom` | `true` |
| `use_robokit` | `true` |
| `use_slam` | `true` |
| `use_rtabmap` | `false` |
| `use_nav` | `false` |
| `localization_backend` | `rtabmap` |
| `rtabmap_db` | `flexiv_amr_nav2/maps/rtabmap_backup.db` |
| `map` | `flexiv_amr_nav2/maps/supermarket.yaml` |
| `use_moveit` | `false` |
| `use_rviz` | `false` |
| `mock_arms` | `false` |
| `joint_control_mode` | `position` |
| `joint_stiffness_ratio` | `1.0` |
| `max_contact_torque` | `0.0` |
| `enable_waist_driver` | `false` |

### Start-up is staged

Nodes come up on timers, so **wait ~30 s before judging anything**:

| Stage | Delay |
|---|---|
| Scan merger | 3 s |
| EKF | 4 s |
| SLAM Toolbox | 8 s |
| Localization | 10 s |
| RTAB-Map mapping | 15 s |
| **Nav2** | **20 s** |
| MoveIt | 6 s |

> `ROS 2 does not validate undeclared top-level launch arguments.` A typo like
> `use_navigation:=true` is silently ignored and you get the defaults. If a
> flag seems to have no effect, check its spelling against the table above.

---

## 7. Multi-machine: RViz on a laptop

Both machines need the same `ROS_DOMAIN_ID`. Add to `~/.bashrc` on **both**:

```bash
export ROS_DOMAIN_ID=42
```

**Multicast discovery does not cross subnet boundaries.** If the robot and
laptop are on different subnets — easy to miss with a `/23` mask, where the
third octet differs — they will never discover each other even with matching
domain IDs. Fix it by naming peers explicitly, on both machines:

```bash
export ROS_DOMAIN_ID=42
export ROS_STATIC_PEERS=<other machine's IP>
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

`ROS_STATIC_PEERS` is semicolon-separated and accepts a hostname, an IPv4 or
IPv6 address, or a subnet such as `192.168.0.0/24`.

Verify from the laptop:

```bash
echo $ROS_DOMAIN_ID
ros2 node list          # should show the robot's nodes
ros2 topic hz /joint_states
```

Then run RViz locally against the robot's data:

```bash
ros2 launch aico2_moveit_config moveit_rviz.launch.py
```

If the robot model does not appear, the laptop is missing meshes or
`/robot_description`. Check `ros2 topic echo /robot_description --once | head`.

---

## 8. Debugging playbook

### Start here, always

**Look at the physical lights before touching any configuration.**

- **Chassis strip slow-blinking red** → E-stop or chassis error. Release the
  E-stops, press **Reset** to clear chassis errors. No ROS topic will tell you
  this if `flexivamr` is missing.
- **Arm motion bar red** → arm fault. `ros2 service call /left_arm/clear_fault std_srvs/srv/Trigger`.

A chassis in E-stop **accepts every `/cmd_vel` packet and ignores it**. The
signature is Nav2 looking perfectly healthy: non-zero `/cmd_vel`, the robot
creeping a few centimetres, and `number_of_recoveries` climbing as the progress
checker times out every ~10 s.

### "Goal was rejected" immediately

`bt_navigator` is configured but not activated. Run the lifecycle loop in
[§5.5](#55-nav2). The `unconfigured [1]` node is the blocker.

### Nav2 reports success but the robot never moved

Check `navigation_time`. Under ~100 ms means the goal was already inside
`xy_goal_tolerance: 0.30` and the goal checker fired on the first tick. Place
the goal more than 0.35 m away.

### `/cmd_vel` is non-zero but the robot does not move

Walk the chain — each stage either prints a Twist or hangs, and hanging is
itself the answer:

```bash
ros2 topic echo /cmd_vel_nav --once        # controller_server output
ros2 topic echo /cmd_vel_smoothed --once   # velocity_smoother output
ros2 topic echo /cmd_vel --once            # collision_monitor output, to chassis
```

The chain is:

```
controller_server ─cmd_vel_nav─> velocity_smoother ─cmd_vel_smoothed─> collision_monitor ─cmd_vel─> chassis
```

| Observation | Cause |
|---|---|
| Non-zero at `cmd_vel_smoothed`, zero at `cmd_vel` | `collision_monitor` is stopping you |
| Non-zero all the way to `cmd_vel`, robot still | Chassis is inhibited (E-stop/fault), or its own obstacle zones are limiting it |
| `cmd_vel_nav` silent | DWB is producing nothing; the recoveries are genuine |

**Teleop bypasses `collision_monitor`** — it publishes straight to `/cmd_vel`.
So "teleop works but Nav2 doesn't" points at the monitor or at Nav2 itself, not
at the chassis link.

### collision_monitor

```bash
ros2 topic echo /collision_monitor/collision_points_marker --once
ros2 topic echo /polygon_stop --once
```

A populated `points` array means it is seeing obstacles inside the 0.30 m
circle. Note `stop_pub_timeout: 2.0` — once tripped it publishes zero for two
full seconds, so intermittent detections produce a robot that crawls.

Its source is `/scan/merged` with `source_timeout: 5.0`. **A stale source is
not ignored** — it forces a STOP action labelled `"invalid source"` and zeroes
the output indefinitely. So no `/scan/merged` means a permanently held robot.

There is no `state_topic` published by default, so
`/collision_monitor_state` does not exist unless you set that parameter.

### `/scan/merged` is silent

The merger blocks on every topic in `laserscan_topics`. Check each input:

```bash
ros2 topic hz /scan/nav
ros2 topic hz /scan/avoid
ros2 topic hz /scan/depth
```

A topic that is advertised but never publishes starves the merger.

### Arm goals fail with CONTROL_FAILED

1. Is the driver `active`? `ros2 lifecycle get /left_arm/left_arm_driver`
2. Is it faulted? `ros2 topic echo --once /left_arm/fault`
3. Is the motion bar in **Auto (Remote)**? Manual mode caps TCP at 250 mm/s and
   needs the enabling button held.
4. Did the goal exceed a joint limit? The driver logs
   `goal tolerance not reached: <joint> is X.XX° from target`.

### Nodes exist but publish nothing

Almost always a missing vendor SDK swallowed at import. See
[§3](#prerequisites). `ros2 node list` and `ros2 topic list` both lie here —
use `ros2 topic hz`.

### A transform is missing

```bash
ros2 run tf2_tools view_frames          # writes frames.pdf
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
```

Only one node may publish `map → odom`. Running SLAM Toolbox and RTAB-Map
localization together is the usual cause of a fighting transform — hence
`use_slam:=false` whenever `use_nav:=true`.

`/amr/actual_velocity` has **two** publishers and last writer wins; one of them
echoes the command rather than a measurement. Use `/odom_raw`.

### Useful one-liners

```bash
ros2 node list
ros2 node info /left_arm/left_arm_driver
ros2 topic list
ros2 topic info /cmd_vel --verbose        # names every publisher and subscriber
ros2 param list /collision_monitor
ros2 param get /controller_server FollowPath.max_vel_x
ros2 param set /left_arm/left_arm_driver default_max_joint_vel 0.6
ros2 lifecycle nodes
ros2 action list
ros2 interface show nav2_msgs/action/NavigateToPose
```

`ros2 topic info --verbose` is the fastest way to find an unexpected second
publisher on a topic.

---

## 9. Known issues

Tracked in **[`docs/open_issues.md`](docs/open_issues.md)**, with what was
measured, what the code says, and what is still hypothesis. Summary:

1. **Execution smoothness — resolved, with one knob left.** Motion used to read
   as three distinct segments. Cause was `AddTimeOptimalParameterization`:
   time-optimal means saturated acceleration, so the velocity profile is a
   trapezoid whose acceleration steps discontinuously at the corners.
   `AddRuckigTrajectorySmoothing` plus jerk limits fixed it.
   The remaining knob is the call rate, which is a U-curve with a minimum
   around 50 Hz — 2 Hz is badly jerky and 190 Hz is jerky again:
   `ros2 param set /left_arm/left_arm_driver trajectory_send_rate_hz 50.0`
   Do not narrow `default_max_joint_vel` / `_acc` toward the trajectory's own
   profile — that was tried and made it jerky again. The controller needs the
   headroom; see [`docs/open_issues.md`](docs/open_issues.md) issue 1.
2. **Up to 3 s dead time after a move**, from the goal settle timeout.
3. **Chassis status is invisible.** `status_monitor` needs `flexivamr`, gives up
   permanently if its first connect fails, and nothing consumes `/amr/status`,
   `/amr/emergency` or `/amr/blocked` even when it works. This is why a chassis
   E-stop reads as a Nav2 failure — check the physical light strip.
4. Smaller: `longest_valid_segment_fraction: 0.005` is unusually tight;
   `both_arms` plans in joint space but has no IK, so no pose goals or RViz
   marker (KDL cannot solve a two-tip group); four `.obj` visual meshes and six
   `.mtl` files are absent, so the URDF points those visuals at collision STLs.

### Other documentation

| File | Contents |
|---|---|
| [`docs/porting.md`](docs/porting.md) | Moving the workspace to a new machine |
| [`docs/open_issues.md`](docs/open_issues.md) | Open issues with measurements |
| [`docs/joint_state_architecture.md`](docs/joint_state_architecture.md) | How `/joint_states` is assembled |
| [`docs/rdk_version_and_compliance.md`](docs/rdk_version_and_compliance.md) | RDK version constraints, compliance modes |
| [`docs/moveit_status.md`](docs/moveit_status.md) | MoveIt configuration state |

Vendor manuals: *AICO2 Series User Manual* (hardware, safety, indicators),
*Flexiv Elements User Manual* (teach pendant software), *Flexiv Rizon User
Manual* (arm), *FMR 300 Series User Manual* (chassis).
