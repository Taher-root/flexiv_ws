#!/usr/bin/env bash
# ============================================================================
# AICO2 / AICO 2-4CE full stack setup on a fresh Ubuntu 24.04 machine.
#
# Prerequisite: ROS 2 Jazzy desktop must already be installed and sourced.
#   https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
#   echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
#
# Installs all system and Python dependencies, builds ROS packages and the
# standalone RT server.
#
# Usage:
#   bash setup_new_machine.sh          # original AICO2 (RDK v1.9)
#   bash setup_new_machine.sh --ce     # AICO 2-4CE   (RDK v1.9.3)
#
# Run as your regular user, NOT root (the script calls sudo where needed).
# ============================================================================
set -euo pipefail

CE=0
for arg in "$@"; do
    case "$arg" in
        --ce) CE=1 ;;
        *) echo "Unknown arg: $arg"; exit 1 ;;
    esac
done

step() { printf '\n\033[1;36m=== %s ===\033[0m\n' "$1"; }

if [[ $CE -eq 1 ]]; then
    RDK_TAG="v1.9.3"
    echo "Setting up for AICO 2-4CE (RDK $RDK_TAG)"
else
    RDK_TAG="v1.9"
    echo "Setting up for original AICO2 (RDK $RDK_TAG)"
fi

# ============================================================================
# 1. Check ROS 2 Jazzy is available
# ============================================================================
step "Checking ROS 2 Jazzy"

if [[ -z "${ROS_DISTRO:-}" ]]; then
    if [[ -f /opt/ros/jazzy/setup.bash ]]; then
        source /opt/ros/jazzy/setup.bash
    else
        echo "ROS 2 Jazzy is not installed."
        echo "Install it first: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html"
        exit 1
    fi
fi

echo "ROS_DISTRO=$ROS_DISTRO"

# ============================================================================
# 2. System dependencies
# ============================================================================
step "Installing system dependencies"

sudo apt install -y \
    build-essential cmake git python3-pip python3-colcon-common-extensions \
    python3-rosdep python3-vcstool \
    libspdlog-dev libfmt-dev \
    libpcl-all-dev \
    ros-jazzy-rmw-cyclonedds-cpp

# ============================================================================
# 3. ROS 2 package dependencies
# ============================================================================
step "Installing ROS 2 package dependencies"

sudo apt install -y \
    ros-jazzy-moveit \
    ros-jazzy-moveit-servo \
    ros-jazzy-moveit-planners-ompl \
    ros-jazzy-moveit-simple-controller-manager \
    ros-jazzy-realsense2-camera \
    ros-jazzy-realsense2-description \
    ros-jazzy-depthimage-to-laserscan \
    ros-jazzy-rtabmap-odom \
    ros-jazzy-rtabmap-slam \
    ros-jazzy-rtabmap-sync \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-proc \
    ros-jazzy-laser-geometry \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    ros-jazzy-xacro \
    ros-jazzy-apriltag-ros \
    ros-jazzy-opennav-docking

# ============================================================================
# 4. Python dependencies
# ============================================================================
step "Installing Python dependencies"

pip3 install --user \
    "flexivrdk==${RDK_TAG#v}" \
    fastapi uvicorn numpy opencv-python

# ============================================================================
# 5. Initialize rosdep
# ============================================================================
step "Initializing rosdep"

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# ============================================================================
# 6. Clone workspace (if not already present)
# ============================================================================
step "Setting up workspace"

WS="$HOME/flexiv_ws"
if [ ! -d "$WS/.git" ]; then
    echo "Clone the workspace manually via SFTP or git, then re-run this script."
    echo "Expected location: $WS"
    echo ""
    echo "If using git:"
    echo "  git clone <your-repo-url> $WS"
    echo ""
    echo "Once cloned, re-run this script with --skip-ros"
    exit 0
fi

cd "$WS"

# ============================================================================
# 7. Build ROS 2 workspace
# ============================================================================
step "Building ROS 2 workspace"

cd "$WS"
rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true

colcon build 2>&1 | tail -5
source "$WS/install/setup.bash"

if ! grep -q "source $WS/install/setup.bash" ~/.bashrc 2>/dev/null; then
    echo "source $WS/install/setup.bash" >> ~/.bashrc
fi

# ============================================================================
# 8. Install Flexiv RDK (standalone, for rt_server)
# ============================================================================
step "Installing Flexiv RDK standalone (for rt_server)"

export RDK_TAG="$RDK_TAG"
bash "$WS/src/aico2_rt_control/scripts/install_rdk.sh" --standalone

# ============================================================================
# 9. Build standalone rt_server
# ============================================================================
step "Building standalone rt_server"

bash "$WS/src/aico2_rt_control/scripts/noros.sh" bash -c "
    cd $WS/src/aico2_rt_control/standalone
    rm -rf build && mkdir build && cd build
    cmake .. -DCMAKE_PREFIX_PATH=\$HOME/rdk_standalone
    make -j\$(nproc)
"

# ============================================================================
# 10. Summary
# ============================================================================
step "Setup complete"

cat <<EOF

AICO2 stack is ready. Key locations:
  Workspace:    $WS
  RDK install:  ~/rdk_standalone
  rt_server:    $WS/src/aico2_rt_control/standalone/build/rt_server

Quick test (source a new terminal first):
  source /opt/ros/jazzy/setup.bash
  source $WS/install/setup.bash

To run the full system:
  ros2 launch flexiv_amr_bringup full_system.launch.py

To run VR teleop:
  # Terminal 1 (ROS-free):
  bash $WS/src/aico2_rt_control/scripts/noros.sh
  sudo $WS/src/aico2_rt_control/standalone/build/rt_server <SERIAL> --gripper Flexiv-GN01

  # Terminal 2 (ROS):
  ros2 launch flexiv_amr_bringup teleop.launch.py arm:=left use_vr:=true

EOF

if [[ $CE -eq 1 ]]; then
cat <<EOF
AICO 2-4CE notes:
  - Use flexiv_amr_bringup_ce / flexiv_amr_description_ce packages
  - Update serial numbers in:
      $WS/src/flexiv_amr_bringup_ce/launch/arms.launch.py
      $WS/src/flexiv_amr_bringup_ce/launch/teleop.launch.py
  - RDK v1.9.3 installed

  ros2 launch flexiv_amr_bringup_ce full_system.launch.py
  ros2 launch flexiv_amr_bringup_ce teleop.launch.py arm:=left use_vr:=true

EOF
fi
