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
#   bash src/aico2_rt_control/scripts/install_rdk.sh --force       # redo a bad prefix
#   bash src/aico2_rt_control/scripts/install_rdk.sh --standalone  # see below
#
# --standalone builds the plain archive with Flexiv's own vendored dependency
# versions, into a SEPARATE prefix, with ROS 2 deliberately not sourced. Use it
# because the ros2-jazzy archive stack-smashes in the Robot constructor against
# the Fast-DDS 2.14.6 / Fast-CDR 2.2.7 that Jazzy currently ships -- Flexiv's own
# basics1_display_robot_states fails identically, so it is the archive, not this
# workspace. The plain archive cannot be linked into anything that also links
# rclcpp, but rt_hold_probe does not, so the RT timing measurement can proceed.
#
# Idempotent apart from --force, which removes the prefix and the clone first.

set -euo pipefail

FORCE=0
STANDALONE=0
for arg in "$@"; do
    case "$arg" in
        --force) FORCE=1 ;;
        --standalone) STANDALONE=1 ;;
        *) printf 'unknown argument: %s\n' "$arg" >&2; exit 2 ;;
    esac
done

# The standalone variant must not see ROS 2 at all. Sourcing ROS 2 exports
# CMAKE_PREFIX_PATH, and find_package searches the cache variable first but the
# environment one too -- so /opt/ros/jazzy wins for anything the prefix has not
# been searched for yet, and the build silently links Jazzy's Fast-DDS again,
# which is the exact thing this variant exists to avoid. Scrubbing a shell by
# hand is unreliable when a dotfile sources ROS, so re-exec with those variables
# removed instead. ROS_SCRUBBED guards against looping.
if [[ " $* " == *" --standalone "* && -z "${ROS_SCRUBBED:-}" ]]; then
    exec env \
        -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
        -u CMAKE_MODULE_PATH -u LD_LIBRARY_PATH -u PKG_CONFIG_PATH \
        -u PYTHONPATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
        -u RMW_IMPLEMENTATION -u AMENT_PYTHON_EXECUTABLE \
        ROS_SCRUBBED=1 bash "$0" "$@"
fi

# Resolve defaults only after parsing, so --standalone can pick different ones.
# The two variants must not share a prefix: one needs Fast-DDS inside it and the
# other needs Fast-DDS absent from it.
RDK_TAG="${RDK_TAG:-v1.9}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
JOBS="${JOBS:-$(nproc)}"
if [[ $STANDALONE -eq 1 ]]; then
    PREFIX="${PREFIX:-$HOME/rdk_standalone}"
    SRC="${SRC:-$HOME/flexiv_rdk_standalone}"
else
    PREFIX="${PREFIX:-$HOME/rdk_install}"
    SRC="${SRC:-$HOME/flexiv_rdk}"
fi

die() { printf '\nERROR: %s\n' "$1" >&2; exit 1; }
step() { printf '\n=== %s ===\n' "$1"; }

step "Preconditions"
if [[ $STANDALONE -eq 0 ]]; then
    [[ -r "$ROS_SETUP" ]] || die "no ROS 2 at $ROS_SETUP (override with ROS_SETUP=)"
fi
for pkg in libspdlog-dev libfmt-dev; do
    dpkg -s "$pkg" >/dev/null 2>&1 || die "$pkg missing: sudo apt install libspdlog-dev libfmt-dev
  The v1.9 static archive references spdlog and fmt without carrying them."
done
command -v cmake >/dev/null || die "cmake missing: sudo apt install build-essential cmake"
echo "variant       $([[ $STANDALONE -eq 1 ]] && echo 'standalone (plain archive, no ROS 2)' || echo 'ros2-jazzy archive')"
[[ $STANDALONE -eq 0 ]] && echo "ROS 2 setup   $ROS_SETUP"
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
    if [[ $STANDALONE -eq 0 ]] \
        && [[ -d "$PREFIX/share/fastrtps" || -d "$PREFIX/lib/cmake/fastcdr" ]]; then
        die "$PREFIX contains its own Fast-DDS/Fast-CDR, which will shadow ROS 2's.
  That is the signature of build_and_install_dependencies.sh having been run.
  Re-run this script with --force."
    fi
    # A previous install of a different RDK version leaves its library behind:
    # `cmake --install` overwrites files it owns but never removes the ones it
    # does not. v1.9 installs libflexiv_rdk.a, v1.9.4+ installs
    # libflexiv_rdk.so, and ending up with both in one lib/ is a trap -- the
    # CMake config points at the right one, but a stale .so of an incompatible
    # version sitting on the loader path is a confusing failure waiting to
    # happen. Remove the other layout's artifacts rather than warning about
    # them, since nothing in this prefix is hand-made.
    shopt -s nullglob
    stale=("$PREFIX"/lib/libflexiv_rdk.so*)
    shopt -u nullglob
    if (( ${#stale[@]} )); then
        echo "removing ${#stale[@]} shared-library file(s) from a newer RDK:"
        printf '  %s\n' "${stale[@]}"
        rm -f "${stale[@]}"
    fi
    echo "prefix exists and looks clean; continuing"
fi

if [[ $STANDALONE -eq 1 ]]; then
    step "Not sourcing ROS 2 (standalone)"
    echo "ROS variables scrubbed; the prefix supplies Fast-DDS/Fast-CDR"
    echo "ROS_DISTRO=${ROS_DISTRO:-unset}  CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH:-unset}"
else
step "Sourcing ROS 2"
# set -u would trip over ROS's own scripts.
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u
echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
[[ -n "${ROS_DISTRO:-}" ]] || die "sourcing $ROS_SETUP did not set ROS_DISTRO"
fi

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

if [[ $STANDALONE -eq 1 ]]; then
    step "Dependencies (full set, including Flexiv's Fast-CDR and Fast-DDS)"
    deps="$SRC/thirdparty/build_and_install_dependencies.sh"
else
step "Dependencies (Boost, SpaceVecAlg, RBDyn only)"
# NOT build_and_install_dependencies.sh: that one also installs Fast-CDR and
# Fast-DDS, whose versions do not match the ros2-jazzy archive.
deps="$SRC/thirdparty/build_and_install_dependencies_not_in_ros2.sh"
fi
[[ -f "$deps" ]] || die "missing $deps -- wrong tag?"
( cd "$SRC/thirdparty" && bash "$deps" "$PREFIX" "$JOBS" )

step "Configuring"
rm -rf "$SRC/build"
mkdir -p "$SRC/build"
log="$SRC/build/configure.log"
jazzy_flag=ON
extra=()
if [[ $STANDALONE -eq 1 ]]; then
    jazzy_flag=OFF
    # Search only the prefix, and ignore $ENV{CMAKE_PREFIX_PATH} even if
    # something put ROS back on it.
    extra+=("-DCMAKE_PREFIX_PATH=$PREFIX"
            "-DCMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH=OFF")
fi
( cd "$SRC/build" && cmake .. \
    -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DRDK_SUPPORT_ROS2_JAZZY=$jazzy_flag \
    "${extra[@]}" ) | tee "$log"

if [[ $STANDALONE -eq 1 ]]; then
    step "Checking Fast-DDS/Fast-CDR came from the prefix"
    grep -E "Found (fastrtps|fastcdr)" "$log" || die "cmake found neither fastrtps nor fastcdr"
    grep -E "Found (fastrtps|fastcdr)" "$log" | grep -q "$PREFIX" \
        || die "Fast-DDS/Fast-CDR did not resolve under $PREFIX.
  Was ROS 2 sourced in this shell? The standalone build needs it absent."
    echo "both resolve under the prefix -- correct for standalone"
else
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
fi

step "Building and installing"
( cd "$SRC/build" && cmake --build . --target install --config Release -j "$JOBS" )

step "Done"
lib="$(find "$PREFIX/lib" -maxdepth 1 -name 'libflexiv_rdk.*' -printf '%f\n' 2>/dev/null | tr '\n' ' ')"
echo "installed: ${lib:-nothing found}"
if [[ $STANDALONE -eq 1 ]]; then
cat <<TXT

Next (no ROS 2 in this shell):
  cd ~/flexiv_ws/src/aico2_rt_control/standalone
  cmake -S . -B build -DCMAKE_PREFIX_PATH=$PREFIX
  cmake --build build -j
  ./build/rt_hold_probe Rizon4-063352

Sanity-check the library first with Flexiv's own example, which is the cleanest
test of whether this archive works at all:
  cd $SRC/example && cmake -S . -B build -DCMAKE_PREFIX_PATH=$PREFIX
  cmake --build build -j && ./build/basics1_display_robot_states Rizon4-063352

Stop the Python arm driver first -- one RDK session per robot. E-stop released,
motion bar in Auto (Remote). The arm should not move.
TXT
else
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
fi
