# Rizon 10s VR Teleop Workspace

Standalone ROS 2 workspace for the Flexiv Rizon 10s arm with RT control and MoveIt.

## Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy (`sudo apt install ros-jazzy-desktop`)
- MoveIt 2 (`sudo apt install ros-jazzy-moveit`)
- Build tools: `sudo apt install build-essential cmake libspdlog-dev libfmt-dev`
- Python RDK wheel: `pip install flexivrdk==1.9.3`

## Setup

```bash
git clone <this-repo> ~/rizon-10s
cd ~/rizon-10s
git checkout rizon10s-clean

# Clone flexiv_description (URDF/meshes)
git clone https://github.com/flexivrobotics/flexiv_description.git src/flexiv_description

# Install the C++ RDK (v1.9.3, standalone build)
bash src/rizon10s_rt_control/scripts/noros.sh
# Inside the noros shell:
bash src/rizon10s_rt_control/scripts/install_rdk.sh --standalone
exit

# Build ROS packages
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

# Build rt_server (must be in a ROS-free shell)
bash src/rizon10s_rt_control/scripts/noros.sh
cd src/rizon10s_rt_control/standalone
mkdir -p build && cd build
cmake .. -DCMAKE_PREFIX_PATH=$HOME/rdk_standalone
make -j$(nproc)
```

## Moving the Arm with MoveIt

Robot must be in **Auto (Remote)** mode in Flexiv Elements.

**Terminal 1** — RT server (ROS-free shell):
```bash
bash ~/rizon-10s/src/rizon10s_rt_control/scripts/noros.sh
cd ~/rizon-10s/src/rizon10s_rt_control/standalone/build
sudo ./rt_server Rizon10s-062545
```

**Terminal 2** — RT bridge:
```bash
source ~/rizon-10s/install/setup.bash
ros2 run rizon10s_rt_control rt_bridge --ros-args -r __ns:=/arm -r joint_states:=/joint_states
```

**Terminal 3** — MoveIt + RViz:
```bash
source ~/rizon-10s/install/setup.bash
ros2 launch rizon10s_moveit_config move_group.launch.py
```

In RViz, drag the interactive marker to a target pose and click **Plan & Execute**.

## Notes

- rt_server needs `sudo` for the 1 kHz RT scheduler. To avoid sudo:
  `sudo setcap cap_sys_nice=eip standalone/build/rt_server`
- The NRT Python driver and rt_server are mutually exclusive (one RDK session per robot).
- Arm IP: `192.168.100.1`, Serial: `Rizon10s-062545`
- RDK v1.9.3 is pinned — the robot rejects v1.9 and v1.9.4.1.
