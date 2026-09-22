#!/usr/bin/env python3
"""Cross-check this MoveIt config against the URDF and the arm drivers.

Pure file parsing -- no ROS, no build, no robot. Catches the class of mistake
that otherwise shows up as "planning works but execution finds no controller",
which is a slow thing to debug on hardware:

  - SRDF naming links or joints the URDF does not have, or naming a fixed joint
    as if it were actuated
  - controller joint lists that disagree with the drivers' joint_names, in
    content or in order (the drivers reject a trajectory missing any joint)
  - controller names that do not resolve to the action the drivers actually
    serve, which is the /<controller_name>/<action_ns> trap
  - joint_limits missing a planning-group joint, or exceeding the URDF's limits
  - kinematics or OMPL entries missing for a group

Run from the workspace root:

    python3 src/aico2_moveit_config/scripts/check_config.py
"""
from __future__ import annotations

import argparse
import re
import sys
import xml.etree.ElementTree as ET

import yaml

_ACTUATED = ("revolute", "continuous", "prismatic")


class Checker:
    def __init__(self):
        self.failures = []

    def check(self, ok, label, detail=""):
        mark = "  ok  " if ok else " FAIL "
        print(f"[{mark}] {label}")
        if not ok:
            if detail:
                print(f"         {detail}")
            self.failures.append(label)
        return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", default="src",
                    help="workspace src directory (default: src)")
    args = ap.parse_args()
    src = args.src.rstrip("/")
    cfg = f"{src}/aico2_moveit_config/config"
    c = Checker()

    urdf = open(f"{src}/flexiv_amr_description/urdf/AICO2-Rizon4.urdf",
                encoding="utf-8").read()
    urdf_links = set(re.findall(r'<link name="([^"]+)"', urdf))
    urdf_joints = {m.group(1): m.group(2) for m in
                   re.finditer(r'<joint name="([^"]+)"\s+type="([^"]+)"', urdf)}
    srdf = ET.parse(f"{cfg}/aico2.srdf").getroot()

    srdf_links = set()
    for chain in srdf.iter("chain"):
        srdf_links.update({chain.get("base_link"), chain.get("tip_link")})
    for dis in srdf.iter("disable_collisions"):
        srdf_links.update({dis.get("link1"), dis.get("link2")})
    missing = sorted(srdf_links - urdf_links)
    c.check(not missing, "SRDF links all exist in the URDF", f"missing {missing}")

    srdf_joints = ({j.get("name") for j in srdf.iter("joint")} |
                   {p.get("name") for p in srdf.iter("passive_joint")})
    missing = sorted(srdf_joints - set(urdf_joints))
    c.check(not missing, "SRDF joints all exist in the URDF", f"missing {missing}")
    nonact = sorted(j for j in srdf_joints
                    if urdf_joints.get(j) not in _ACTUATED)
    c.check(not nonact, "SRDF joints are all actuated types", f"{nonact}")

    ctrl = yaml.safe_load(open(f"{cfg}/moveit_controllers.yaml", encoding="utf-8"))
    scm = ctrl["moveit_simple_controller_manager"]
    for side, path in (
        ("left", f"{src}/aico2_left_arm_driver/config/left_arm_hardware.yaml"),
        ("right", f"{src}/aico2_right_arm_driver/config/right_arm_hardware.yaml"),
    ):
        drv = yaml.safe_load(open(path, encoding="utf-8"))
        drv_joints = drv[next(iter(drv))]["ros__parameters"]["joint_names"]
        name = f"{side}_arm"
        c.check(name in scm["controller_names"], f"controller {name!r} declared",
                f"have {scm['controller_names']}")
        if name not in scm:
            continue
        c.check(scm[name]["joints"] == drv_joints,
                f"controller {name!r} joints match the driver, in order",
                f"moveit={scm[name]['joints']}\n         driver={drv_joints}")
        action = f"/{name}/{scm[name]['action_ns']}"
        c.check(action == f"/{side}_arm/follow_joint_trajectory",
                f"controller {name!r} resolves to the driver's action", action)
    c.check(ctrl.get("moveit_manage_controllers") is False,
            "moveit_manage_controllers is false (no controller_manager exists)")

    limits = yaml.safe_load(
        open(f"{cfg}/joint_limits.yaml", encoding="utf-8"))["joint_limits"]
    group_joints = {f"{side}_joint{i}"
                    for side in ("Left", "Right") for i in range(1, 8)}
    missing = sorted(group_joints - set(limits))
    c.check(not missing, "joint_limits covers all planning-group joints",
            f"missing {missing}")
    over = []
    for joint, lim in limits.items():
        m = re.search(r'<joint name="' + joint +
                      r'"[^>]*>.*?<limit[^/]*velocity="([\d.]+)"', urdf, re.S)
        if m and lim.get("max_velocity", 0) > float(m.group(1)) + 1e-6:
            over.append(f"{joint}: {lim['max_velocity']} > urdf {m.group(1)}")
    c.check(not over, "joint_limits velocities within the URDF limits",
            "; ".join(over))

    groups = {g.get("name") for g in srdf.findall("group")}
    chain_groups = {g.get("name") for g in srdf.findall("group")
                    if g.find("chain") is not None}
    kin = yaml.safe_load(open(f"{cfg}/kinematics.yaml", encoding="utf-8"))
    c.check(chain_groups <= set(kin), "kinematics.yaml covers every chain group",
            f"groups={sorted(chain_groups)} kinematics={sorted(kin)}")

    ompl = yaml.safe_load(open(f"{cfg}/ompl_planning.yaml", encoding="utf-8"))
    c.check(groups <= set(ompl), "ompl_planning.yaml covers every group",
            f"groups={sorted(groups)}")
    bad = []
    for group in sorted(groups & set(ompl)):
        found = re.findall(r"joints\(([^)]*)\)",
                           ompl[group].get("projection_evaluator", ""))
        for joint in (found[0].split(",") if found else []):
            if joint and joint not in urdf_joints:
                bad.append(f"{group}: {joint}")
    c.check(not bad, "ompl projection_evaluator joints exist", "; ".join(bad))

    print()
    if c.failures:
        print(f"{len(c.failures)} failure(s): {'; '.join(c.failures)}")
        return 1
    print("all cross-checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
