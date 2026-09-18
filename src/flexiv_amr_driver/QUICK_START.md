# Quick Start Guide - Robokit Integration

## 🚀 Quick Test (5 Minutes)

### Step 1: Transfer to Thor Machine

```bash
# On your Windows machine, copy flexiv_ws to Thor
scp -r C:\Users\tkapadia\Downloads\flexiv_ws ubuntu@<thor-ip>:~/
```

### Step 2: Build on Thor

```bash
# SSH to Thor
ssh ubuntu@<thor-ip>

# Navigate to workspace
cd ~/flexiv_ws

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Build (only flexiv_amr_driver changed)
colcon build --packages-select flexiv_amr_driver

# Source workspace
source install/setup.bash
```

### Step 3: Test Connection

```bash
# Test Robokit TCP connection
python3 src/flexiv_amr_driver/test_robokit_connection.py

# Expected output:
# ✓ Connected to STATE API at 192.168.1.110:19204
# ✓ Received location data
# ✓ Connected to CTRL API at 192.168.1.110:19205
# ✓ Sent stop command
# ✓ All tests passed!
```

### Step 4: Launch Robokit Driver

```bash
# Launch with Robokit velocity control
ros2 launch flexiv_amr_driver amr_driver_robokit.launch.py
```

### Step 5: Test with Teleop

```bash
# In another terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Press keys to move:
# i = forward
# k = stop
# j = turn left
# l = turn right

# Expected: SMOOTH motion (no jerking!)
```

## 📊 Compare Original vs Robokit

### Test Original (Baseline):

```bash
ros2 launch flexiv_amr_driver amr_driver.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Note: Jerky, stuttering motion
```

### Test Robokit (New):

```bash
ros2 launch flexiv_amr_driver amr_driver_robokit.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Note: Smooth, responsive motion
```

## 🔧 Troubleshooting

### Connection Failed?

```bash
# Check AMR is reachable
ping 192.168.1.110

# Check if Robokit ports are open
nc -zv 192.168.1.110 19204
nc -zv 192.168.1.110 19205
```

### Robot Not Moving?

```bash
# Check if commands are being sent
ros2 topic echo /cmd_vel

# Check if controller is running
ros2 node list | grep robokit

# Check controller logs
ros2 node info /robokit_velocity_controller
```

### Need to Revert?

```bash
# Just use original launch file
ros2 launch flexiv_amr_driver amr_driver.launch.py
# All original code is untouched!
```

## 📁 What Changed?

### New Files (Safe to Delete if Needed):
- `flexiv_amr_driver/robokit_protocol.py`
- `flexiv_amr_driver/robokit_velocity_controller.py`
- `launch/amr_driver_robokit.launch.py`
- `config/amr_params_robokit.yaml`
- `test_robokit_connection.py`
- `ROBOKIT_README.md`
- `QUICK_START.md` (this file)

### Modified Files:
- `setup.py` (added one line for new entry point)

### Untouched Files:
- `velocity_controller.py` ✅
- `odometry_publisher.py` ✅
- `status_monitor.py` ✅
- `amr_driver.launch.py` ✅
- `amr_params.yaml` ✅

## ✅ Success Criteria

After following this guide, you should see:

1. ✅ Test script passes all checks
2. ✅ Robokit driver launches without errors
3. ✅ Teleop keyboard controls robot smoothly
4. ✅ No jerky or stuttering motion
5. ✅ Robot responds quickly to commands

## 🎯 Next Steps

Once velocity control is working:

1. **Test with full navigation stack:**
   ```bash
   ros2 launch flexiv_amr_bringup mapping.launch.py
   ```

2. **Email Marc for LiDAR/IMU request IDs**

3. **Implement Phase 2: High-rate odometry**

4. **Implement Phase 3: Native sensors**

---

**Estimated Time:** 5-10 minutes
**Difficulty:** Easy
**Risk:** Low (original code untouched)
