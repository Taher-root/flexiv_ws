#!/usr/bin/env bash
# Run a command, or an interactive shell, with ROS 2 removed from the environment.
#
# The standalone RDK build must not see ROS 2: find_package consults the
# environment CMAKE_PREFIX_PATH as well as the cache variable, so a shell with
# ROS 2 sourced links Jazzy's Fast-DDS 2.14.6 instead of the prefix's 2.6.10 --
# and that combination stack-smashes in the Robot constructor. Scrubbing by hand
# is unreliable when a dotfile sources ROS, so this is the single definition of
# what "ROS-free" means here; install_rdk.sh re-execs through it.
#
#   bash scripts/noros.sh                      # interactive ROS-free shell
#   bash scripts/noros.sh cmake -S . -B build  # one command
#
# Only this process tree is affected; the calling shell is untouched.

set -euo pipefail

ROS_VARS=(
    AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_MODULE_PATH
    LD_LIBRARY_PATH PKG_CONFIG_PATH PYTHONPATH
    ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY
    ROS_AUTOMATIC_DISCOVERY_RANGE RMW_IMPLEMENTATION AMENT_PYTHON_EXECUTABLE
)

# Print the list when asked, so install_rdk.sh does not duplicate it.
if [[ "${1:-}" == "--print-vars" ]]; then
    printf '%s\n' "${ROS_VARS[@]}"
    exit 0
fi

unset_args=()
for v in "${ROS_VARS[@]}"; do
    unset_args+=(-u "$v")
done

# cmake find_package also searches parent dirs of PATH entries, so
# /opt/ros/jazzy/bin on PATH lets cmake find ament packages under
# /opt/ros/jazzy/share/ even with CMAKE_PREFIX_PATH unset. Strip it.
clean_path=""
IFS=: read -ra _dirs <<< "${PATH:-}"
for _d in "${_dirs[@]}"; do
    [[ "$_d" == /opt/ros/* ]] && continue
    clean_path="${clean_path:+$clean_path:}$_d"
done

if [[ $# -eq 0 ]]; then
    printf 'ROS-free shell. ROS_DISTRO and CMAKE_PREFIX_PATH are unset here.\n'
    printf 'Exit to return to your normal environment.\n\n'
    exec env "${unset_args[@]}" NOROS=1 PATH="$clean_path" bash --norc -i
fi

exec env "${unset_args[@]}" NOROS=1 PATH="$clean_path" "$@"
