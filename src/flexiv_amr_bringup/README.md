# flexiv_amr_bringup

Top-level launch files for the Flexiv AICO2 (dual Rizon arms on a Seer/Robokit AMR).

## Structure

Launch files compose upward — each layer includes the one below it rather than
restating its nodes, so a node's parameters are defined in exactly one place.

```
full_system.launch.py            arms + hardware + EKF + (SLAM | RTAB-Map | Nav2)
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

Subsystem launches live in their own packages (`flexiv_amr_nav2` for
`ekf`/`slam`/`localization`/`navigation`/`rtabmap_mapping`), and are reused here.

## Common commands

```bash
# Hardware only — verify chassis, sensors and TF, no mapping/navigation
ros2 launch flexiv_amr_bringup hardware_test.launch.py

# Mapping (slam_toolbox). Drive around, then save:
#   ros2 run nav2_map_server map_saver_cli -f <path>/my_map
ros2 launch flexiv_amr_bringup mapping.launch.py
ros2 launch flexiv_amr_bringup mapping.launch.py use_robokit:=true use_rviz:=false

# Navigation against a saved map (set the initial pose in RViz first)
ros2 launch flexiv_amr_bringup navigation.launch.py map:=/path/to/map.yaml

# Everything: arms + chassis + sensors + mapping
ros2 launch flexiv_amr_bringup full_system.launch.py

# Full system variants
ros2 launch flexiv_amr_bringup full_system.launch.py use_slam:=false use_rtabmap:=true
ros2 launch flexiv_amr_bringup full_system.launch.py use_slam:=false use_nav:=true map:=/path/to/map.yaml
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
| `use_visual_odom` | hardware_test, full_system | `true` | RTAB-Map RGBD visual odometry |
| `laserscan_topics` | hardware_test, sensors | nav+avoid+depth | Scans merged into `/scan/merged` |
| `merger_delay` | hardware_test, sensors | `0.0` | Delay before starting the scan merger |
| `mock_arms` | full_system, arms_with_navigation | `false` | Run the arm drivers without flexivrdk |
| `enable_waist_driver` | full_system, arms, arms_with_navigation | `false` | Publish real `AGV_Jiont1/2` (see `aico2_waist_driver/README.md`) |
| `use_slam` / `use_rtabmap` / `use_nav` | full_system | `true`/`false`/`false` | Which mapping or navigation stack to start |
| `use_rviz` | most | varies | Launch RViz2 |

`full_system.launch.py` turns the top camera on with the rest of the cameras and
adds `/scan/top` to the merged scan; it also owns the staged start-up delays
(scan merger at 3 s, EKF at 4 s, SLAM at 8 s, RTAB-Map at 15 s, AMCL at 10 s,
Nav2 at 20 s), since those exist only when the whole system starts at once.

## Known gaps

- **No map yaml is checked in.** `maps/supermarket.pgm` exists in
  `flexiv_amr_nav2` but its `supermarket.yaml` (resolution, origin,
  occupancy thresholds) does not, so navigation modes need `map:=` pointing at
  a real map until it is restored.
- **No RViz configs are checked in.** RViz launches with its default view;
  the older `mapping.rviz` / `navigation.rviz` referenced by earlier versions
  of these launch files were never committed.

## Troubleshooting

```bash
ping 192.168.1.110                  # AMR chassis reachable
ros2 topic echo /amr/status          # chassis driver alive
lsusb | grep Intel                   # cameras enumerated
ros2 topic hz /camera/camera/color/image_raw
ros2 topic echo /map --once          # map loaded (navigation mode)
ros2 topic echo /amcl_pose           # localization converged
ros2 lifecycle get /left_arm/left_arm_driver
```

Visual odometry needs texture and light — it degrades badly on blank walls.
