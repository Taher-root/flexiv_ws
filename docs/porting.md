# Porting this workspace to a new machine

Verified statically on 2026-09-26. `scripts/smoke_test.sh` runs every check
below and exits non-zero if the port is incomplete.

## 1. Prerequisites the workspace cannot install for you

ROS 2 **Jazzy** (`ros-jazzy-desktop`), plus two vendor Python SDKs that are
**not rosdep keys** and must be installed by hand:

| module | used by | note |
|---|---|---|
| `flexivrdk` | `aico2_left_arm_driver`, `aico2_waist_driver`, the `rdk_*.py` scripts | Flexiv RDK 1.9.0 wheel, matched to controller software v3.11. 2.x does **not** support Rizon — see `rdk_version_and_compliance.md`. |
| `flexivamr` | `flexiv_amr_driver/velocity_controller.py`, `status_monitor.py` | AMR vendor SDK. Only needed with `use_robokit:=false`. |

Both import lazily, so the workspace builds without them; the affected nodes
fail at runtime with an import error that names the module.

A machine that only *views* (RViz, MoveIt panel) needs neither.

## 2. Get the code

```bash
git clone -b <branch> <repo> ~/flexiv_ws
cd ~/flexiv_ws
```

`ira_laser_tools` is vendored at `src/ira_laser_tools_jazzy/src/ira_laser_tools/`
— nested two levels down, which colcon finds recursively. Do not flatten it.

## 3. Dependencies

```bash
sudo rosdep init          # first time on the machine only
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Every `package.xml` was audited on 2026-09-26 against what the code actually
imports and what the launch files actually start. Nine packages were missing
declarations at that point — including `rtabmap_slam` and `rtabmap_sync`, which
`rtabmap_mapping.launch.py` and `rtabmap_localization.launch.py` both start,
and `opennav_docking`, which `navigation.launch.py` starts. On a fresh machine
those would have installed nothing and failed at launch. Keep the audit honest:

```bash
# re-run the check after adding any import or any Node(package=...)
./scripts/smoke_test.sh --no-build
```

## 4. Build

```bash
colcon build --symlink-install
source install/setup.bash
```

A viewer-only machine needs just two packages, and neither needs a vendor SDK:

```bash
colcon build --packages-select flexiv_amr_description aico2_moveit_config
```

## 5. Verify, headless

```bash
./scripts/smoke_test.sh
```

Six checks, none of which touch a robot, a camera or a display:

1. `rosdep` resolves every declared key (dry run)
2. `colcon build` succeeds
3. every mesh the URDF references exists on disk — this is the check that would
   have caught the five missing visual meshes that cost most of 2026-09-26
4. every launch file parses and its arguments are declared, via `--show-args`
5. unit tests pass (20 at time of writing)
6. mock bringup: both arm drivers reach `active` with `mock_hardware:=true`,
   `/joint_states` publishes, and the `follow_joint_trajectory` action is served

## 6. Multi-machine setup

Each machine needs, in `~/.bashrc`:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_STATIC_PEERS=<THE OTHER MACHINE'S IP>
[ -f ~/flexiv_ws/install/setup.bash ] && source ~/flexiv_ws/install/setup.bash
```

`ROS_STATIC_PEERS` is required when the machines are on **different subnets**:
multicast does not cross a subnet boundary, so `ROS_AUTOMATIC_DISCOVERY_RANGE`
(default `SUBNET`) finds nothing even though ping works. Same subnet, and it
can be omitted. The value is the *other* machine, so it differs per machine.

Both machines must run the same URDF. RViz resolves `package://` mesh paths
against its **own** filesystem and loads `robot_description` from its **own**
copy, so a stale URDF on the viewer draws links that TF never places:

```bash
md5sum install/flexiv_amr_description/share/flexiv_amr_description/urdf/AICO2-Rizon4.urdf
```

Must match on every machine.
