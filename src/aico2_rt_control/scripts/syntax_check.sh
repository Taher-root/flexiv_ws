#!/usr/bin/env bash
# Type-check the RDK programs without linking, and without a robot.
#
# The RDK library is a prebuilt binary for the target, so these cannot be built
# on a development machine -- but the *headers* are plain source, so the
# compiler can still verify every call signature, the RobotAdapter interface
# and the RtExecutor template instantiation. That catches the entire class of
# error that would otherwise be discovered on the robot, at speed, next to a
# moving arm.
#
#   bash scripts/syntax_check.sh                       # clones headers if needed
#   RDK_HEADERS=~/flexiv_rdk/include bash scripts/syntax_check.sh
#
# Needs g++ and Eigen (apt install libeigen3-dev).

set -euo pipefail
HERE="$(dirname "$(readlink -f "$0")")"
PKG="$(dirname "$HERE")"
RDK_HEADERS="${RDK_HEADERS:-}"
EIGEN="${EIGEN:-/usr/include/eigen3}"

if [[ -z "$RDK_HEADERS" ]]; then
    for cand in "$HOME/flexiv_rdk_standalone/include" "$HOME/flexiv_rdk/include"; do
        [[ -d "$cand" ]] && RDK_HEADERS="$cand" && break
    done
fi
[[ -d "$RDK_HEADERS" ]] || {
    echo "No RDK headers found. Set RDK_HEADERS=<path>/include, or clone:" >&2
    echo "  git clone --depth 1 -b v1.9 https://github.com/flexivrobotics/flexiv_rdk.git" >&2
    exit 1
}
[[ -d "$EIGEN" ]] || { echo "Eigen not at $EIGEN (apt install libeigen3-dev)" >&2; exit 1; }

echo "RDK headers  $RDK_HEADERS"
echo "Eigen        $EIGEN"
fail=0
for src in "$PKG"/src/*.cpp; do
    printf '%-20s ' "$(basename "$src" .cpp)"
    if g++ -fsyntax-only -std=c++17 -Wall -Wextra -Wpedantic \
        -I "$RDK_HEADERS" -I "$PKG/include" -I "$EIGEN" "$src"; then
        echo "clean"
    else
        echo "FAILED"
        fail=1
    fi
done
exit "$fail"
