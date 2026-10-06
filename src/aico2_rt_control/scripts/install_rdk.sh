#!/usr/bin/env bash
# Install the Flexiv RDK C++ library at the version these arms accept.
#
# Pinned to the v1.9 tag: robot software v3.11 rejects a v1.9.4.1 client with
# "Version of this client is incompatible with robot", while the v1.9 Python
# wheel drives these arms in production. See README.md for the measurements.
#
# Exists as a script because the steps are order-sensitive in two ways that are
# silent when wrong -- ROS 2 must be sourced before the dependency script, and
# the ros2-jazzy archive must be selected explicitly -- and because pasting the
# sequence into a terminal lets `apt` swallow the following lines.
#
#   bash src/aico2_rt_control/scripts/install_rdk.sh
#   bash src/aico2_rt_control/scripts/install_rdk.sh --force     # redo a bad prefix
#
# Idempotent apart from --force, which removes the prefix and the clone first.

set -euo pipefail

RDK_TAG="${RDK_TAG:-v1.9}"
PREFIX="${PREFIX:-$HOME/rdk_install}"
SRC="${SRC:-$HOME/flexiv_rdk}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
JOBS="${JOBS:-$(nproc)}"
FORCE=0
[[ "${1:-}" == "--force" ]] && FORCE=1

die() { printf '\nERROR: %s\n' "$1" >&2; exit 1; }
step() { printf '\n=== %s ===\n' "$1"; }

step "Preconditions"
[[ -r "$ROS_SETUP" ]] || die "no ROS 2 at $ROS_SETUP (override with ROS_SETUP=)"
for pkg in libspdlog-dev libfmt-dev; do
    dpkg -s "$pkg" >/dev/null 2>&1 || die "$pkg missing: sudo apt install libspdlog-dev libfmt-dev
  The v1.9 static archive references spdlog and fmt without carrying them."
done
command -v cmake >/dev/null || die "cmake missing: sudo apt install build-essential cmake"
echo "ROS 2 setup   $ROS_SETUP"
echo "spdlog/fmt    present"
echo "tag           $RDK_TAG"
echo "prefix        $PREFIX"
echo "source        $SRC"

if [[ $FORCE -eq 1 ]]; then
    step "Removing previous prefix and clone (--force)"
    rm -rf "$PREFIX" "$SRC"
elif [[ -d "$PREFIX" ]]; then
    # A prefix built by the wrong dependency script carries Fast-CDR 1.0.28 and
    # Fast-DDS 2.6.10, which shadow ROS 2's and are the cause of several hundred
    # undefined eprosima:: symbols. Reconfiguring does not undo that.
    if [[ -d "$PREFIX/share/fastrtps" || -d "$PREFIX/lib/cmake/fastcdr" ]]; then
        die "$PREFIX contains its own Fast-DDS/Fast-CDR, which will shadow ROS 2's.
  That is the signature of build_and_install_dependencies.sh having been run.
  Re-run this script with --force."
    fi
    echo "prefix exists and looks clean; continuing"
fi

step "Sourcing ROS 2"
# set -u would trip over ROS's own scripts.
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u
echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
[[ -n "${ROS_DISTRO:-}" ]] || die "sourcing $ROS_SETUP did not set ROS_DISTRO"

step "Fetching RDK $RDK_TAG"
if [[ -d "$SRC/.git" ]]; then
    have="$(git -C "$SRC" describe --tags --exact-match 2>/dev/null || echo unknown)"
    [[ "$have" == "$RDK_TAG" ]] \
        || die "$SRC is at '$have', not $RDK_TAG. Re-run with --force."
    echo "already at $RDK_TAG"
else
    git clone --depth 1 -b "$RDK_TAG" \
        https://github.com/flexivrobotics/flexiv_rdk.git "$SRC"
fi

step "Dependencies (Boost, SpaceVecAlg, RBDyn only)"
# NOT build_and_install_dependencies.sh: that one also installs Fast-CDR and
# Fast-DDS, whose versions do not match the ros2-jazzy archive.
deps="$SRC/thirdparty/build_and_install_dependencies_not_in_ros2.sh"
[[ -f "$deps" ]] || die "missing $deps -- wrong tag?"
( cd "$SRC/thirdparty" && bash "$deps" "$PREFIX" "$JOBS" )

step "Configuring"
rm -rf "$SRC/build"
mkdir -p "$SRC/build"
log="$SRC/build/configure.log"
( cd "$SRC/build" && cmake .. \
    -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DRDK_SUPPORT_ROS2_JAZZY=ON ) | tee "$log"

step "Checking which Fast-DDS/Fast-CDR were selected"
# The whole point of the ROS 2 path. If these resolve under the prefix, the
# wrong dependency script ran and the link will fail later with a wall of
# undefined eprosima:: symbols -- better to stop here.
grep -E "Found (fastrtps|fastcdr)" "$log" || die "cmake found neither fastrtps nor fastcdr"
if grep -E "Found (fastrtps|fastcdr)" "$log" | grep -q "$PREFIX"; then
    die "Fast-DDS/Fast-CDR resolved under $PREFIX instead of ROS 2.
  Re-run with --force."
fi
echo "both resolve outside the prefix -- correct"

step "Building and installing"
( cd "$SRC/build" && cmake --build . --target install --config Release -j "$JOBS" )

step "Done"
lib="$(find "$PREFIX/lib" -maxdepth 1 -name 'libflexiv_rdk.*' -printf '%f\n' 2>/dev/null | tr '\n' ' ')"
echo "installed: ${lib:-nothing found}"
cat <<TXT

Next:
  cd ~/flexiv_ws
  colcon build --packages-select aico2_rt_control \\
      --cmake-args -DCMAKE_PREFIX_PATH=$PREFIX
  source install/setup.bash
  ros2 run aico2_rt_control rt_hold_probe Rizon4-063352

Stop the Python arm driver first -- one RDK session per robot. E-stop released,
motion bar in Auto (Remote). The arm should not move.

Do NOT set LD_LIBRARY_PATH to the prefix: assigning it drops ROS 2's lib
directory and breaks ros2 with a missing librcl_action.so.
TXT
