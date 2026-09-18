# flexiv_amr_bringup

Top-level launch files for the Flexiv AICO2 (dual Rizon arms on a Seer/Robokit AMR).

## Structure

Launch files compose upward — each layer includes the one below it rather than
restating its nodes, so a node's parameters are defined in exactly one place.

```
full_system.launch.py            arms + hardware + EKF + (SLAM | RTAB-Map | localization+Nav2)
├─ arms.launch.py                both Rizon lifecycle drivers, waist, merger
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

Subsystem launches live in `flexiv_amr_nav2` (`ekf`, `slam`, `localization`,
`rtabmap_localization`, `navigation`, `rtabmap_mapping`) and are reused here.

### Localization backend

`navigation.launch.py` and `full_system.launch.py use_nav:=true` pick one
localization backend via `localization_backend` (default `rtabmap`) — exactly
one of these owns the `map -> odom` TF and `/map`:

- **`rtabmap`** (default) — `rtabmap_localization.launch.py` localizes against
  `maps/rtabmap.db` using lidar + visual features. Never writes to the
  database (`Mem/IncrementalMemory` and `delete_db_on_start` are forced off in
  that launch file regardless of `rtabmap_params.yaml`). Override the database
  with `rtabmap_db:=/path/to/other.db`.
- **`amcl`** — `localization.launch.py`, laser-only, against `map:=` (a map
  yaml, default `maps/supermarket.yaml`).

`rtabmap_mapping.launch.py` (used by `use_rtabmap:=true` for *mapping*, not
localization) is a separate file — see Known gaps below, it currently shares a
quirk with the localization path that's worth fixing before relying on it.

## Common commands

```bash
# Hardware only — verify chassis, sensors and TF, no mapping/navigation
ros2 launch flexiv_amr_bringup hardware_test.launch.py

# Mapping (slam_toolbox). Drive around, then save:
#   ros2 run nav2_map_server map_saver_cli -f <path>/my_map
ros2 launch flexiv_amr_bringup mapping.launch.py
ros2 launch flexiv_amr_bringup mapping.launch.py use_robokit:=true use_rviz:=false

# Navigation, localizing against maps/rtabmap.db (default backend)
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
| `enable_waist_driver` | full_system, arms, arms_with_navigation | `false` | Publish real `AGV_Jiont1/2` (see `aico2_waist_driver/README.md`) |
| `use_slam` / `use_rtabmap` | full_system | `true` / `false` | Mapping backend: SLAM Toolbox or RTAB-Map |
| `use_nav` | navigation, full_system | `false` (`navigation.launch.py` implies true) | Start localization + Nav2 |
| `localization_backend` | navigation, full_system | `rtabmap` | `rtabmap` (against `rtabmap_db:=`) or `amcl` (against `map:=`) |
| `rtabmap_db` | navigation, full_system | `maps/rtabmap.db` | Database RTAB-Map localizes against |
| `map` | navigation, full_system | `maps/supermarket.yaml` | Map yaml AMCL localizes against |
| `use_rviz` | most | varies | Launch RViz2 |

`full_system.launch.py` turns the top camera on with the rest of the cameras and
adds `/scan/top` to the merged scan; it also owns the staged start-up delays
(scan merger at 3 s, EKF at 4 s, SLAM at 8 s, RTAB-Map mapping at 15 s,
localization at 10 s, Nav2 at 20 s), since those exist only when the whole
system starts at once.

## Known gaps

- **`rtabmap_mapping.launch.py` (the mapping backend, `use_rtabmap:=true`) is
  configured for localization, not mapping.** `rtabmap_params.yaml` sets
  `Mem/IncrementalMemory: 'false'`, so as things stand this launch file will
  localize against whatever `database_path` points at rather than grow a new
  map — the opposite of what `use_rtabmap:=true` promises for a *mapping* run.
  That file's `database_path` is also a single-user absolute path
  (`/home/vivangupta/...rtabmap_backup.db`) that won't resolve on another
  checkout. Fix both before trusting `use_rtabmap:=true` for actual mapping.
  (`rtabmap_localization.launch.py`, the new localization path, is unaffected —
  it forces its own `Mem/IncrementalMemory`/`database_path`/`delete_db_on_start`
  regardless of this file.)
- **Visual odometry from the bottom camera is dead-coded off in `ekf.yaml`**
  (`odom1: /camera/odom` is commented out — "quality=0, publishes null poses").
  Only the top camera's visual odometry (`odom2`) is fused, and only when
  `use_top_camera:=true`. On the single-camera default path (`hardware_test`,
  `mapping`, `navigation` without `full_system`), visual odometry is computed
  but never reaches the EKF.
- **No RViz configs are checked in.** RViz launches with its default view;
  the older `mapping.rviz` / `navigation.rviz` referenced by earlier versions
  of these launch files were never committed.
- **`realsense.launch.py` / `visual_odometry.launch.py`** (`flexiv_amr_sensors`)
  are not included by any other launch file in this repo — if nothing outside
  git invokes them directly, they're dead. `realsense.launch.py` is also
  missing a `PythonLaunchDescriptionSource` wrapper around its include, and
  `visual_odometry.launch.py` uses the ROS 1 `rtabmap_ros` package name. Left
  as-is pending confirmation of real usage.

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
