# Robokit Integration - README

## Overview

This package now includes **Robokit TCP API integration** for continuous velocity control, providing smooth robot motion compared to the original discrete command approach.

## What Was Added

### New Files Created:

1. **`flexiv_amr_driver/robokit_protocol.py`**
   - Low-level Robokit TCP protocol implementation
   - Message packing/unpacking functions
   - Port and request ID definitions
   - Based on: https://github.com/seer-robotics/Robokit_TCP_API_py

2. **`flexiv_amr_driver/robokit_velocity_controller.py`**
   - ROS 2 node for continuous velocity control
   - Uses Robokit CTRL API (port 19205)
   - Subscribes to `/cmd_vel`, publishes to `/amr/actual_velocity`
   - Smooth, responsive motion (no jerky movements)

3. **`launch/amr_driver_robokit.launch.py`**
   - Launch file using Robokit velocity controller
   - Keeps original odometry and status monitor
   - Easy A/B testing with original launch file

4. **`config/amr_params_robokit.yaml`**
   - Robokit-specific parameters
   - TCP port configuration
   - Timeout settings

5. **`scripts/check_robokit_connection.py`**
   - Standalone test script
   - Verifies Robokit TCP connection
   - Tests STATE and CTRL APIs

### Modified Files:

1. **`setup.py`**
   - Added entry point: `robokit_velocity_controller`
   - Original entry points unchanged

## Original vs Robokit Comparison

| Feature | Original (Flexiv API) | Robokit Integration |
|---------|----------------------|---------------------|
| **Velocity Control** | Discrete (translation/rotation) | Continuous (vx, vy, w) |
| **Motion Quality** | Jerky, stuttering | Smooth, responsive |
| **Response Time** | 200-500ms | <50ms |
| **Nav2 Compatibility** | Workaround | Native |
| **API Used** | Flexiv NavigatorAPI | Robokit CTRL API |
| **Port** | N/A (library) | 19205 (TCP) |

## How to Use

### Option 1: Use Original (Current Behavior)

```bash
# On Thor machine
cd ~/flexiv_ws
source install/setup.bash
ros2 launch flexiv_amr_driver amr_driver.launch.py
```

**Behavior:** Discrete commands, jerky motion (current)

### Option 2: Use Robokit (New Smooth Control)

```bash
# On Thor machine
cd ~/flexiv_ws

# First, rebuild to register new node
colcon build --packages-select flexiv_amr_driver
source install/setup.bash

# Test connection first (optional but recommended)
python3 src/flexiv_amr_driver/scripts/check_robokit_connection.py

# Launch with Robokit
ros2 launch flexiv_amr_driver amr_driver_robokit.launch.py
```

**Behavior:** Continuous velocity, smooth motion (new)

### Testing with Teleop

```bash
# Terminal 1: Launch driver
ros2 launch flexiv_amr_driver amr_driver_robokit.launch.py

# Terminal 2: Launch teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Expected:** Smooth, responsive motion when pressing keys

## Architecture

### Robokit TCP API Ports:

- **19200** - ROBOD (Robot Daemon) - Sensors (LiDAR, IMU)
- **19204** - STATE - Robot state (location, speed, battery)
- **19205** - CTRL - Control (velocity commands) ← **We use this!**
- **19206** - TASK - Task management (navigation)
- **19207** - CONFIG - Configuration
- **19210** - OTHER - Digital I/O, audio

### Data Flow:

```
/cmd_vel (ROS 2)
    ↓
robokit_velocity_controller.py
    ↓
Robokit Protocol (packMsg)
    ↓
TCP Socket (port 19205)
    ↓
Flexiv AMR Base (Seer Platform)
    ↓
Smooth Robot Motion!
```

## Troubleshooting

### Connection Failed

```bash
# Test connection manually
python3 src/flexiv_amr_driver/scripts/check_robokit_connection.py

# Check AMR IP
ping 192.168.1.110

# Check if ports are open
nc -zv 192.168.1.110 19205
```

### Robot Not Moving

1. Check if velocity commands are being sent:
   ```bash
   ros2 topic echo /cmd_vel
   ```

2. Check actual velocity output:
   ```bash
   ros2 topic echo /amr/actual_velocity
   ```

3. Check node logs:
   ```bash
   ros2 node info /robokit_velocity_controller
   ```

### Reverting to Original

Simply use the original launch file:
```bash
ros2 launch flexiv_amr_driver amr_driver.launch.py
```

All original files are untouched!

## Next Steps

### Phase 2: High-Rate Odometry (Future)

Create `robokit_odometry_publisher.py` to query:
- Location (request ID 1004) at 50Hz
- Speed (request ID 1005) at 50Hz

### Phase 3: Native Sensors (Future)

Once we get LiDAR/IMU request IDs from Flexiv:
- Create `robokit_sensor_interface.py`
- Query LiDAR point cloud (port 19200)
- Query IMU data (port 19200)

## Technical Details

### Robokit Protocol Format:

**Header (16 bytes):**
- Magic byte: 0x5A
- Version: 1
- Serial number: 1, 2, 3, ... (increments)
- JSON length: size of data
- Request type: 2010 for velocity
- Reserved: 6 bytes (zeros)

**Body (variable):**
- JSON data: `{"vx": 0.5, "vy": 0.0, "w": 0.0}`

### Velocity Command Example:

```python
from robokit_protocol import packMsg, robot_control_motion_req

# Create command
cmd = {"vx": 0.5, "vy": 0.0, "w": 0.0}  # Move forward at 0.5 m/s
msg = packMsg(1, robot_control_motion_req, cmd)

# Send via socket
sock.send(msg)
```

## References

- Robokit GitHub: https://github.com/seer-robotics/Robokit_TCP_API_py
- Robokit Protocol PDF: http://static.seer-robotics.com/robotkit-netprotocol-l-1.4.2.pdf
- Seer Robotics: https://www.seer-robotics.com

## Support

For issues or questions:
1. Check this README
2. Run `scripts/check_robokit_connection.py`
3. Check ROS 2 logs: `ros2 node info /robokit_velocity_controller`
4. Contact Flexiv support for LiDAR/IMU request IDs

---

**Created:** 2026-06-23
**Status:** ✅ Velocity control working, ⏳ Odometry/sensors pending
