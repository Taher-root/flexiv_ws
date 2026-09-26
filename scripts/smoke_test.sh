#!/usr/bin/env bash
# Headless portability check: does this workspace build and run on a fresh
# machine with no robot, no cameras and no display?
#
#   ./scripts/smoke_test.sh            # build + all checks
#   ./scripts/smoke_test.sh --no-build # checks only, against an existing build
#
# Everything here runs with mock_hardware:=true and use_rviz:=false. Nothing
# touches a robot. Exit code 0 means the port is good.
set -uo pipefail
cd "$(dirname "$0")/.."
WS=$(pwd)
PASS=0; FAIL=0
ok()   { echo "  PASS  $1"; PASS=$((PASS+1)); }
bad()  { echo "  FAIL  $1"; FAIL=$((FAIL+1)); }
step() { echo; echo "=== $1"; }

[ -z "${ROS_DISTRO:-}" ] && { echo "source /opt/ros/<distro>/setup.bash first"; exit 1; }
echo "workspace: $WS"
echo "ROS_DISTRO: $ROS_DISTRO"

# ---------------------------------------------------------------- 1. rosdep
step "1. rosdep — every declared dependency resolves"
if command -v rosdep >/dev/null; then
  if rosdep install --from-paths src --ignore-src -r -s >/tmp/rosdep_sim.txt 2>&1; then
    ok "rosdep resolves all keys (dry run)"
  else
    bad "rosdep could not resolve everything — see /tmp/rosdep_sim.txt"
    grep -iE "cannot|error|not found" /tmp/rosdep_sim.txt | head -5 | sed 's/^/        /'
  fi
else
  bad "rosdep not installed"
fi

# ---------------------------------------------------------------- 2. build
if [ "${1:-}" != "--no-build" ]; then
  step "2. colcon build"
  if colcon build --symlink-install >/tmp/build.log 2>&1; then
    ok "all packages built"
  else
    bad "build failed — see /tmp/build.log"; tail -20 /tmp/build.log | sed 's/^/        /'
    echo; echo "PASS=$PASS FAIL=$FAIL"; exit 1
  fi
else
  step "2. colcon build (skipped)"
fi
# shellcheck disable=SC1091
source install/setup.bash

# ---------------------------------------------------------------- 3. files
step "3. every mesh the URDF references exists"
python3 - <<'PY'
import os, sys, xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
share = get_package_share_directory("flexiv_amr_description")
urdf = os.path.join(share, "urdf", "AICO2-Rizon4.urdf")
bad = []
for link in ET.parse(urdf).getroot().findall("link"):
    for kind in ("visual", "collision"):
        for el in link.findall(kind):
            m = el.find("geometry/mesh")
            if m is None: continue
            rel = m.get("filename").replace("package://flexiv_amr_description/", "")
            if not os.path.exists(os.path.join(share, rel)):
                bad.append(f"{link.get('name')} [{kind}] {rel}")
print("MESH_RESULT", "OK" if not bad else "MISSING " + "; ".join(bad))
sys.exit(0 if not bad else 1)
PY
[ $? -eq 0 ] && ok "all URDF meshes resolve" || bad "URDF references missing meshes"

# ---------------------------------------------------------------- 4. launch
step "4. every launch file's arguments are declared and includes resolve"
for lf in $(find src -name "*.launch.py" -not -path "*__pycache__*" | sort); do
  pkg=$(echo "$lf" | cut -d/ -f2)
  name=$(basename "$lf")
  if timeout 60 ros2 launch "$pkg" "$name" --show-args >/dev/null 2>/tmp/la.txt; then
    ok "$pkg/$name"
  else
    bad "$pkg/$name"; head -3 /tmp/la.txt | sed 's/^/        /'
  fi
done

# ---------------------------------------------------------------- 5. tests
step "5. unit tests (no hardware)"
if colcon test --packages-select aico2_left_arm_driver aico2_moveit_config \
     --event-handlers console_cohesion- >/tmp/test.log 2>&1; then
  colcon test-result --all >/tmp/tres.txt 2>&1
  if grep -q "Summary" /tmp/tres.txt && ! grep -qE "[1-9][0-9]* (error|failure)" /tmp/tres.txt; then
    ok "$(grep Summary /tmp/tres.txt | head -1)"
  else
    bad "test failures"; grep -E "error|failure" /tmp/tres.txt | head -5 | sed 's/^/        /'
  fi
else
  bad "colcon test did not run — see /tmp/test.log"
fi

# ------------------------------------------------------- 6. mock bringup
step "6. mock arm bringup — nodes reach 'active' with no robot"
ros2 launch flexiv_amr_bringup arms.launch.py mock_hardware:=true \
  >/tmp/mock.log 2>&1 &
LAUNCH_PID=$!
for _ in $(seq 1 30); do
  sleep 1
  st=$(ros2 lifecycle get /left_arm/left_arm_driver 2>/dev/null | head -1)
  [[ "$st" == active* ]] && break
done
st_l=$(ros2 lifecycle get /left_arm/left_arm_driver 2>/dev/null | head -1)
st_r=$(ros2 lifecycle get /right_arm/right_arm_driver 2>/dev/null | head -1)
[[ "$st_l" == active* ]] && ok "left_arm_driver active" || bad "left_arm_driver: ${st_l:-no response}"
[[ "$st_r" == active* ]] && ok "right_arm_driver active" || bad "right_arm_driver: ${st_r:-no response}"

n=$(timeout 5 ros2 topic echo /joint_states --field name --once 2>/dev/null | grep -c joint)
[ "${n:-0}" -gt 0 ] && ok "/joint_states publishing ($n joints in one message)" \
                    || bad "/joint_states not publishing"
timeout 10 ros2 action list 2>/dev/null | grep -q "/left_arm/follow_joint_trajectory" \
  && ok "follow_joint_trajectory action served" || bad "trajectory action missing"

kill $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null

echo
echo "================================"
echo " PASS: $PASS    FAIL: $FAIL"
echo "================================"
[ "$FAIL" -eq 0 ] || echo "logs: /tmp/rosdep_sim.txt /tmp/build.log /tmp/test.log /tmp/mock.log"
exit $((FAIL > 0))
