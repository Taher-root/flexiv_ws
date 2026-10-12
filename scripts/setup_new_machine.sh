#!/usr/bin/env bash
# ============================================================================
# AICO2 / AICO 2-4CE: build the standalone Flexiv RDK and rt_server.
#
# This script only builds the RDK library and the rt_server binary.
# For all other setup steps (ROS 2, apt/pip deps, colcon build), see the
# instructions printed at the end, or run with --help.
#
# Usage:
#   bash setup_new_machine.sh          # original AICO2 (RDK v1.9)
#   bash setup_new_machine.sh --ce     # AICO 2-4CE   (RDK v1.9.3)
#   bash setup_new_machine.sh --help   # print full setup instructions
#
# Run as your regular user, NOT root (the script calls sudo where needed).
# ============================================================================
set -euo pipefail

CE=0
HELP=0
for arg in "$@"; do
    case "$arg" in
        --ce) CE=1 ;;
        --help|-h) HELP=1 ;;
        *) echo "Unknown arg: $arg"; exit 1 ;;
    esac
done

if [[ $CE -eq 1 ]]; then
    RDK_TAG="v1.9.3"
else
    RDK_TAG="v1.9"
fi

WS="$HOME/flexiv_ws"

# --help: print the full setup guide and exit
if [[ $HELP -eq 1 ]]; then
cat <<EOF
Full setup for a fresh Ubuntu 24.04 machine
============================================

1. Install ROS 2 Jazzy desktop:
   https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

2. Add to ~/.bashrc:
   echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
   source ~/.bashrc

3. Install system dependencies:
   sudo apt update
   sudo apt install -y \\
       build-essential cmake git python3-pip python3-colcon-common-extensions \\
       python3-rosdep python3-vcstool \\
       libspdlog-dev libfmt-dev \\
       libpcl-all-dev \\
       ros-jazzy-rmw-cyclonedds-cpp

4. Install ROS 2 package dependencies:
   sudo apt install -y \\
       ros-jazzy-moveit \\
       ros-jazzy-moveit-servo \\
       ros-jazzy-moveit-planners-ompl \\
       ros-jazzy-moveit-simple-controller-manager \\
       ros-jazzy-realsense2-camera \\
       ros-jazzy-realsense2-description \\
       ros-jazzy-depthimage-to-laserscan \\
       ros-jazzy-rtabmap-odom \\
       ros-jazzy-rtabmap-slam \\
       ros-jazzy-rtabmap-sync \\
       ros-jazzy-nav2-bringup \\
       ros-jazzy-slam-toolbox \\
       ros-jazzy-robot-localization \\
       ros-jazzy-foxglove-bridge \\
       ros-jazzy-joy \\
       ros-jazzy-teleop-twist-joy \\
       ros-jazzy-cv-bridge \\
       ros-jazzy-image-proc \\
       ros-jazzy-laser-geometry \\
       ros-jazzy-pcl-conversions \\
       ros-jazzy-pcl-ros \\
       ros-jazzy-xacro \\
       ros-jazzy-apriltag-ros \\
       ros-jazzy-opennav-docking

5. Install Python dependencies:
   pip3 install --user \\
       "flexivrdk==${RDK_TAG#v}" \\
       fastapi uvicorn numpy opencv-python

6. Clone the workspace to ~/flexiv_ws (via SFTP or git).

7. Initialize rosdep:
   sudo rosdep init    # skip if already done
   rosdep update

8. Build the standalone RDK and rt_server (THIS SCRIPT):
   cd ~/flexiv_ws
   bash scripts/setup_new_machine.sh$([ $CE -eq 1 ] && echo " --ce")

9. Build the ROS 2 workspace:
   cd ~/flexiv_ws
   source /opt/ros/jazzy/setup.bash
   rosdep install --from-paths src --ignore-src -r -y
   colcon build

10. Source the workspace (and add to bashrc):
    source ~/flexiv_ws/install/setup.bash
    echo "source ~/flexiv_ws/install/setup.bash" >> ~/.bashrc

EOF
    if [[ $CE -eq 1 ]]; then
cat <<EOF
AICO 2-4CE notes:
  - Use flexiv_amr_bringup_ce / flexiv_amr_description_ce packages
  - Update serial numbers in:
      ~/flexiv_ws/src/flexiv_amr_bringup_ce/launch/arms.launch.py
      ~/flexiv_ws/src/flexiv_amr_bringup_ce/launch/teleop.launch.py

EOF
    fi
    exit 0
fi

# ============================================================================
# Actual work: RDK install + rt_server build
# ============================================================================
step() { printf '\n\033[1;36m=== %s ===\033[0m\n' "$1"; }

if [[ $CE -eq 1 ]]; then
    echo "Building for AICO 2-4CE (RDK $RDK_TAG)"
else
    echo "Building for original AICO2 (RDK $RDK_TAG)"
fi

if [ ! -d "$WS/src" ]; then
    echo "Workspace not found at $WS (no src/ directory)"
    echo "Copy or clone it first, then re-run. Use --help for full instructions."
    exit 1
fi

# ---------------------------------------------------------------------------
# 1. Install Flexiv RDK (standalone, for rt_server)
# ---------------------------------------------------------------------------
step "Installing Flexiv RDK standalone"

export RDK_TAG="$RDK_TAG"
bash "$WS/src/aico2_rt_control/scripts/install_rdk.sh" --standalone

# ---------------------------------------------------------------------------
# 2. Build standalone rt_server
# ---------------------------------------------------------------------------
step "Building standalone rt_server"

bash "$WS/src/aico2_rt_control/scripts/noros.sh" bash -c "
    cd $WS/src/aico2_rt_control/standalone
    rm -rf build && mkdir build && cd build
    cmake .. -DCMAKE_PREFIX_PATH=\$HOME/rdk_standalone
    make -j\$(nproc)
"

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
step "Done"

cat <<EOF

rt_server built successfully.

  RDK install:  ~/rdk_standalone
  rt_server:    $WS/src/aico2_rt_control/standalone/build/rt_server

Run --help for the full setup instructions (deps, colcon build, etc.):
  bash $WS/scripts/setup_new_machine.sh --help

VR teleop quick start:
  # Terminal 1 (ROS-free shell):
  bash $WS/src/aico2_rt_control/scripts/noros.sh
  sudo $WS/src/aico2_rt_control/standalone/build/rt_server <SERIAL> --gripper Flexiv-GN01

  # Terminal 2 (ROS):
  ros2 launch flexiv_amr_bringup teleop.launch.py arm:=left use_vr:=true

EOF

if [[ $CE -eq 1 ]]; then
cat <<EOF
AICO 2-4CE: use flexiv_amr_bringup_ce instead:
  ros2 launch flexiv_amr_bringup_ce full_system.launch.py
  ros2 launch flexiv_amr_bringup_ce teleop.launch.py arm:=left use_vr:=true

EOF
fi
