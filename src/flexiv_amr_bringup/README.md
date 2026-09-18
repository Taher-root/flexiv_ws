# Flexiv AMR Bringup

Master launch files for Flexiv AICO2 AMR with ROS 2 navigation.

## Launch Files

### 1. Hardware Test
Test hardware without navigation:
```bash
ros2 launch flexiv_amr_bringup hardware_test.launch.py
```

**What it launches:**
- AMR driver (velocity control, odometry, status monitoring)
- RealSense camera (RGB-D + IMU)
- Visual odometry
- Depth-to-laserscan conversion
- Robot state publisher (URDF)

**Use this to:**
- Verify AMR connection
- Test camera
- Check sensor data flow

---

### 2. Mapping Mode
Create a map of the environment:
```bash
ros2 launch flexiv_amr_bringup mapping.launch.py
```

**What it launches:**
- Everything from hardware_test
- EKF sensor fusion
- SLAM Toolbox (mapping)
- RViz (optional)

**Workflow:**
1. Launch mapping mode
2. Drive robot around using teleop or joystick
3. SLAM builds map in real-time
4. Save map when complete:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/flexiv_ws/maps/my_map
   ```

---

### 3. Navigation Mode
Autonomous navigation with saved map:
```bash
ros2 launch flexiv_amr_bringup navigation.launch.py map:=/path/to/map.yaml
```

**What it launches:**
- Everything from hardware_test
- EKF sensor fusion
- AMCL localization (with map)
- NAV2 stack (path planning, obstacle avoidance)
- RViz (optional)

**Workflow:**
1. Launch navigation mode with your map
2. Set initial pose in RViz (2D Pose Estimate)
3. Set navigation goal in RViz (Nav2 Goal)
4. Robot navigates autonomously

---

## Parameters

### Hardware Test
```bash
# No parameters needed
ros2 launch flexiv_amr_bringup hardware_test.launch.py
```

### Mapping
```bash
# Disable RViz (for headless operation)
ros2 launch flexiv_amr_bringup mapping.launch.py use_rviz:=false
```

### Navigation
```bash
# Use custom map
ros2 launch flexiv_amr_bringup navigation.launch.py \
  map:=/path/to/custom_map.yaml

# Disable RViz
ros2 launch flexiv_amr_bringup navigation.launch.py \
  use_rviz:=false

# Both
ros2 launch flexiv_amr_bringup navigation.launch.py \
  map:=/path/to/custom_map.yaml \
  use_rviz:=false
```

---

## Teleoperation

Control robot manually during mapping:

```bash
# Keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Joystick teleop (if you have a gamepad)
ros2 launch teleop_twist_joy teleop-launch.py
```

---

## Troubleshooting

### AMR not connecting
- Check network: `ping 192.168.1.110`
- Verify Flexiv library: `python3 -c 'import flexivamr'`
- Check driver logs: `ros2 topic echo /amr/status`

### Camera not working
- Check USB connection: `lsusb | grep Intel`
- Test camera: `ros2 launch realsense2_camera rs_launch.py`
- Check topics: `ros2 topic list | grep camera`

### Visual odometry failing
- Ensure good lighting
- Check for textured environment (not blank walls)
- Verify camera topics: `ros2 topic hz /camera/color/image_raw`

### Navigation not working
- Verify map is loaded: `ros2 topic echo /map -n 1`
- Check localization: `ros2 topic echo /amcl_pose`
- Verify costmaps: `ros2 topic list | grep costmap`

---

## System Architecture

```
Hardware Layer:
  ├─ Flexiv AMR (192.168.1.110)
  └─ RealSense D456 (USB)

Driver Layer:
  ├─ velocity_controller (cmd_vel → AMR API)
  ├─ odometry_publisher (dead reckoning)
  └─ status_monitor (battery, emergency, blocked)

Sensor Layer:
  ├─ RealSense driver (RGB-D + IMU)
  ├─ Visual odometry (rtabmap)
  └─ Depth-to-laserscan (fake 2D scan)

Localization Layer:
  ├─ EKF (fuses: dead reckoning + visual odom + IMU)
  ├─ SLAM Toolbox (mapping mode)
  └─ AMCL (navigation mode)

Navigation Layer:
  └─ NAV2 (planning + control + recovery)
```

---

## Dependencies

See `~/flexiv_ws/DEPENDENCIES.md` for complete list.

Quick install:
```bash
cd ~/flexiv_ws
./install_dependencies.sh
```
```

Save (Ctrl+X, Y, Enter)

---

## **Step 6: Update CMakeLists.txt**

```bash
nano CMakeLists.txt
```

Add before `ament_package()`:

```cmake
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)
```

Save (Ctrl+X, Y, Enter)

---

## **Step 7: Build**

```bash
cd ~/flexiv_ws
colcon build --packages-select flexiv_amr_bringup
source install/setup.bash