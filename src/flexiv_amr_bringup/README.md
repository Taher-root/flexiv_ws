# flexiv_amr_bringup

Top-level launch files for the Flexiv AICO2 (dual Rizon arms on a Seer/Robokit AMR).

## Structure

Launch files compose upward — each layer includes the one below it rather than
restating its nodes, so a node's parameters are defined in exactly one place.

```
full_system.launch.py            arms + hardware + EKF + (SLAM | RTAB-Map | localization+Nav2)
├─ arms.launch.py                both Rizon lifecycle drivers, waist, merger*
└─ hardware_test.launch.py       URDF/TF + chassis + sensors
   ├─ display.launch.py          (flexiv_amr_description)  robot_state_publisher
   ├─ amr_driver.launch.py       (flexiv_amr_driver)       chassis + odom + status
   └─ sensors.launch.py          (flexiv_amr_sensors)      LiDARs, IMU, camera(s),
                                                          depth→scan, scan merger

mapping.launch.py                hardware_test + ekf + slam [+ RViz]
navigation.launch.py             hardware_test + ekf + localization + Nav2 [+ RViz]
arms_with_navigation.launch.py   arms + mapping_robokit

mapping_robokit.launch.py        alias: mapping.launch.py use_robokit:=true
```

*`joint_state_merger` runs only while `direct_joint_states:=false` — see below.

Subsystem launches live in `flexiv_amr_nav2` (`ekf`, `slam`, `localization`,
`rtabmap_localization`, `navigation`, `rtabmap_mapping`) and are reused here.

### Localization backend

`navigation.launch.py` and `full_system.launch.py use_nav:=true` pick one
localization backend via `localization_backend` (default `rtabmap`) — exactly
one of these owns the `map -> odom` TF and `/map`:

- **`rtabmap`** (default) — `rtabmap_localization.launch.py` localizes against
  `maps/rtabmap_backup.db` using lidar + visual features. Never writes to the
  database (`Mem/IncrementalMemory` and `delete_db_on_start` are forced off in
  that launch file regardless of `rtabmap_params.yaml`). Override the database
  with `rtabmap_db:=/path/to/other.db`.
- **`amcl`** — `localization.launch.py`, laser-only, against `map:=` (a map
  yaml, default `maps/supermarket.yaml`).

`rtabmap_mapping.launch.py` (used by `use_rtabmap:=true`) is the mapping
counterpart: it forces `Mem/IncrementalMemory: true` and writes to its own
`rtabmap_db` (default `~/.ros/aico2_map.db`), deliberately never the map
navigation localizes against. `rtabmap_params.yaml` holds only the tuning both
modes share — the three mode keys live in the launch files so neither can
silently flip the other's behaviour.

### Retiring joint_state_merger

`direct_joint_states` switches between the two `/joint_states` topologies
(`joint_state_architecture.md` sec 3, sec 8 step 6):

- **`false`** (default) — arms publish `/left_arm/joint_states` and
  `/right_arm/joint_states`; `joint_state_merger` stitches them into a
  16-joint `/joint_states` with the waist **forced to 0.0**.
- **`true`** — each driver publishes only its own joints straight to
  `/joint_states`, the merger is not started, and `robot_state_publisher`
  merges the partial messages by joint name. This is what lets
  `enable_waist_driver:=true` actually reach TF instead of being overwritten
  with the merger's zeros.

Implemented as a launch remap, so flipping it back is one argument. Nothing in
this repo consumes the per-arm topics except the merger, but check anything
outside it (VR teleop, other machines on the ROS domain) before making `true`
the default.

## Common commands

```bash
# Hardware only — verify chassis, sensors and TF, no mapping/navigation
ros2 launch flexiv_amr_bringup hardware_test.launch.py

# Mapping (slam_toolbox). Drive around, then save:
#   ros2 run nav2_map_server map_saver_cli -f <path>/my_map
ros2 launch flexiv_amr_bringup mapping.launch.py
ros2 launch flexiv_amr_bringup mapping.launch.py use_robokit:=true use_rviz:=false

# Navigation, localizing against maps/rtabmap_backup.db (default backend)
ros2 launch flexiv_amr_bringup navigation.launch.py

# Navigation with AMCL instead (laser-only, against a saved map yaml)
ros2 launch flexiv_amr_bringup navigation.launch.py \
    localization_backend:=amcl map:=/path/to/map.yaml

# Everything: arms + chassis + sensors + mapping
ros2 launch flexiv_amr_bringup full_system.launch.py

# Full system variants
ros2 launch flexiv_amr_bringup full_system.launch.py use_slam:=false use_rtabmap:=true
ros2 launch flexiv_amr_bringup full_system.launch.py use_slam:=false use_nav:=true
ros2 launch flexiv_amr_bringup full_system.launch.py use_slam:=false use_nav:=true \
    localization_backend:=amcl map:=/path/to/map.yaml
ros2 launch flexiv_amr_bringup full_system.launch.py mock_arms:=true
ros2 launch flexiv_amr_bringup full_system.launch.py use_camera:=false use_visual_odom:=false

# Arms only
ros2 launch flexiv_amr_bringup arms.launch.py
ros2 launch flexiv_amr_bringup arms.launch.py mock_hardware:=true
```

## Arguments

| Argument | Files | Default | Meaning |
|---|---|---|---|
| `use_robokit` | hardware_test, mapping, navigation | `false` | Robokit TCP velocity controller instead of the plain one |
| `use_camera` | hardware_test, full_system | `true` | RealSense camera(s) + depth-to-laserscan |
| `use_top_camera` | hardware_test, sensors | `false` | Also bring up the head-mounted D456 (adds `/scan/top`) |
| `use_visual_odom` | hardware_test, full_system | `true` | RTAB-Map RGBD visual odometry (fused into EKF — see Known gaps) |
| `laserscan_topics` | hardware_test, sensors | nav+avoid+depth | Scans merged into `/scan/merged` |
| `merger_delay` | hardware_test, sensors | `0.0` | Delay before starting the scan merger |
| `mock_arms` | full_system, arms_with_navigation | `false` | Run the arm drivers without flexivrdk |
| `enable_waist_driver` | full_system, arms, arms_with_navigation | `false` | Publish real `AGV_Joint1/2` (see `aico2_waist_driver/README.md`) |
| `use_slam` / `use_rtabmap` | full_system | `true` / `false` | Mapping backend: SLAM Toolbox or RTAB-Map |
| `use_nav` | navigation, full_system | `false` (`navigation.launch.py` implies true) | Start localization + Nav2 |
| `localization_backend` | navigation, full_system | `rtabmap` | `rtabmap` (against `rtabmap_db:=`) or `amcl` (against `map:=`) |
| `rtabmap_db` | navigation, full_system | `maps/rtabmap_backup.db` | Database RTAB-Map localizes against (mapping writes to `~/.ros/aico2_map.db`) |
| `map` | navigation, full_system | `maps/supermarket.yaml` | Map yaml AMCL localizes against |
| `direct_joint_states` | full_system, arms, arms_with_navigation | `false` | Arms publish straight to `/joint_states`, retiring `joint_state_merger` |
| `use_rviz` | most | varies | Launch RViz2 |

`full_system.launch.py` turns the top camera on with the rest of the cameras and
adds `/scan/top` to the merged scan; it also owns the staged start-up delays
(scan merger at 3 s, EKF at 4 s, SLAM at 8 s, RTAB-Map mapping at 15 s,
localization at 10 s, Nav2 at 20 s), since those exist only when the whole
system starts at once.

## Known gaps

- **`maps/rtabmap_backup.db` is not checked in.** The localization default now
  points at it (it is the good map: 53 MB, 2026-08-31, versus the 1.8 MB
  2026-08-04 `rtabmap.db` that *is* committed). Either commit it or place it in
  `flexiv_amr_nav2/maps/` on the robot, or pass `rtabmap_db:=` explicitly.
- **The top D456 is not mounted or connected**, so `ekf.yaml` fuses only the
  bottom camera's visual odometry (`odom1: /camera/odom`). The `odom2` block and
  the `use_top_camera:=true` plumbing are both in place and commented/off —
  re-enable both once the camera is physically installed.
- **The bottom camera's visual odometry was previously disabled** in `ekf.yaml`
  with "quality=0, publishes null poses". It is enabled again as of 2026-09-18
  because it is the only connected camera; if that symptom returns, check
  `/camera/odom` before suspecting the filter.
- **No RViz configs are checked in.** RViz launches with its default view; the
  older `mapping.rviz` / `navigation.rviz` these launch files once referenced
  were never committed.
- **`AGV_Joint1`/`AGV_Joint2` are misspelled** in the URDF (joint names only —
  the links `AGV_Yaw`/`AGV_Pitch` and all meshes are spelled correctly, so TF
  frames are unaffected). In-repo it is 5 functional occurrences; renaming also
  touches anything outside the repo that looks joints up by name.

## Troubleshooting

```bash
ping 192.168.1.110                  # AMR chassis reachable
ros2 topic echo /amr/status          # chassis driver alive
lsusb | grep Intel                   # cameras enumerated
ros2 topic hz /camera/camera/color/image_raw
ros2 topic echo /map --once          # map loaded (navigation mode)
ros2 topic echo /amcl_pose           # AMCL converged (localization_backend:=amcl)
ros2 topic hz /odometry/filtered     # EKF alive (rtabmap localization needs this)
ros2 lifecycle get /left_arm/left_arm_driver
```

Visual odometry needs texture and light — it degrades badly on blank walls.
