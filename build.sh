#!/usr/bin/env bash
set -euo pipefail

# ── Rizon 10s workspace build script ──
#
# Usage:
#   bash build.sh              # build all packages
#   bash build.sh --deps       # install rosdep dependencies first
#   bash build.sh --clean      # clean build (remove build/ install/ log/)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ── Parse args ──
INSTALL_DEPS=false
CLEAN_BUILD=false
for arg in "$@"; do
    case "$arg" in
        --deps)  INSTALL_DEPS=true ;;
        --clean) CLEAN_BUILD=true ;;
        -h|--help)
            echo "Usage: bash build.sh [--deps] [--clean]"
            echo "  --deps   Install ROS 2 dependencies via rosdep"
            echo "  --clean  Remove build/ install/ log/ before building"
            exit 0
            ;;
        *) echo "Unknown arg: $arg"; exit 1 ;;
    esac
done

# ── Source ROS 2 ──
if [ -z "${ROS_DISTRO:-}" ]; then
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    else
        echo "ERROR: No ROS 2 installation found. Source your ROS 2 setup.bash first."
        exit 1
    fi
    echo "Sourced ROS 2 $ROS_DISTRO"
fi

# ── Install dependencies ──
if $INSTALL_DEPS; then
    echo "Installing rosdep dependencies..."
    sudo rosdep init 2>/dev/null || true
    rosdep update --rosdistro="$ROS_DISTRO"
    rosdep install --from-paths src --ignore-src -r -y
fi

# ── Clean ──
if $CLEAN_BUILD; then
    echo "Cleaning build artifacts..."
    rm -rf build install log
fi

# ── Build ──
echo "Building Rizon 10s workspace..."
colcon build
echo ""
echo "Build complete. Source the workspace:"
echo "  source install/setup.bash"
