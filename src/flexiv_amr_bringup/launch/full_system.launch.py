#!/usr/bin/env python3
"""Complete AICO2 system: arms + chassis + sensors + mapping or navigation.

Composed from the per-subsystem launch files rather than restating them:

  arms.launch.py                  (flexiv_amr_bringup)  both Rizon arms, waist, merger
  hardware_test.launch.py         (flexiv_amr_bringup)  URDF/TF + chassis + sensors
    display.launch.py             (flexiv_amr_description)
    amr_driver.launch.py          (flexiv_amr_driver)
    sensors.launch.py             (flexiv_amr_sensors)
  ekf.launch.py                   (flexiv_amr_nav2)  sensor fusion
  slam.launch.py                  (flexiv_amr_nav2)  SLAM Toolbox mapping    [use_slam]
  rtabmap_mapping.launch.py       (flexiv_amr_nav2)  RTAB-Map mapping        [use_rtabmap]
  rtabmap_localization.launch.py  (flexiv_amr_nav2)  RTAB-Map localization  [use_nav, default]
  localization.launch.py          (flexiv_amr_nav2)  AMCL + map_server      [backend=amcl]
  navigation.launch.py            (flexiv_amr_nav2)  Nav2 stack              [use_nav]

The staged start delays live here, because they are a property of bringing the
whole system up at once, not of the subsystems themselves.

Usage:
  # Mapping with slam_toolbox (default)
  ros2 launch flexiv_amr_bringup full_system.launch.py

  # Mapping with RTAB-Map
  ros2 launch flexiv_amr_bringup full_system.launch.py use_slam:=false use_rtabmap:=true

  # Navigation, localizing against maps/rtabmap_backup.db (default backend)
  ros2 launch flexiv_amr_bringup full_system.launch.py use_slam:=false use_nav:=true

  # Navigation with AMCL instead (laser-only, against a saved map yaml)
  ros2 launch flexiv_amr_bringup full_system.launch.py use_slam:=false use_nav:=true \
      localization_backend:=amcl map:=/path/to/map.yaml

  # Mock arms (no robot connection) / no camera
  ros2 launch flexiv_amr_bringup full_system.launch.py mock_arms:=true
  ros2 launch flexiv_amr_bringup full_system.launch.py use_camera:=false use_visual_odom:=false
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Scans merged into /scan/merged once the head camera is in play.
LASERSCAN_TOPICS = "/scan/nav /scan/avoid /scan/depth /scan/top"

# Staged start-up, seconds after launch.
MERGER_DELAY = 3.0          # TF tree + scan sources up
EKF_DELAY = 4.0             # odom + IMU sources publishing
SLAM_DELAY = 8.0            # arms + EKF + merged scan settled
RTABMAP_MAP_DELAY = 15.0    # both cameras enumerated on USB and streaming
LOCALIZATION_DELAY = 10.0   # odometry/filtered + scans + (for rtabmap) camera up
NAV2_DELAY = 20.0           # localization active, /map and map->odom TF up


def _include(package, launch_file, launch_arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])
        ),
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    use_camera = LaunchConfiguration("use_camera")
    use_visual_odom = LaunchConfiguration("use_visual_odom")
    use_slam = LaunchConfiguration("use_slam")
    use_rtabmap = LaunchConfiguration("use_rtabmap")
    use_nav = LaunchConfiguration("use_nav")
    use_rviz = LaunchConfiguration("use_rviz")
    map_file = LaunchConfiguration("map")
    rtabmap_db = LaunchConfiguration("rtabmap_db")
    localization_backend = LaunchConfiguration("localization_backend")
    mock_arms = LaunchConfiguration("mock_arms")

    nav2_params = PathJoinSubstitution(
        [FindPackageShare("flexiv_amr_nav2"), "config", "nav2_params.yaml"]
    )

    def _nav_backend_is(backend):
        return IfCondition(PythonExpression(
            ["'", use_nav, "' == 'true' and '", localization_backend, f"' == '{backend}'"]
        ))

    return LaunchDescription([
        # --- Arguments ---
        DeclareLaunchArgument("use_camera", default_value="true",
                              description="Launch both RealSense cameras + depth-to-laserscan"),
        DeclareLaunchArgument("use_visual_odom", default_value="true",
                              description="Launch RTAB-Map visual odometry"),
        DeclareLaunchArgument("use_slam", default_value="true",
                              description="Launch SLAM Toolbox for mapping"),
        DeclareLaunchArgument("use_rtabmap", default_value="false",
                              description="Launch RTAB-Map instead of slam_toolbox"),
        DeclareLaunchArgument("use_nav", default_value="false",
                              description="Launch localization + Nav2 stack"),
        DeclareLaunchArgument("localization_backend", default_value="rtabmap",
                              description="'rtabmap' (localize against rtabmap_db, "
                                          "lidar+visual) or 'amcl' (laser-only against map:=)"),
        DeclareLaunchArgument("rtabmap_db",
                              default_value=PathJoinSubstitution([
                                  FindPackageShare("flexiv_amr_nav2"),
                                  "maps", "rtabmap_backup.db",
                              ]),
                              description="RTAB-Map database to localize against "
                                          "(localization_backend:=rtabmap)"),
        DeclareLaunchArgument("map",
                              default_value=PathJoinSubstitution([
                                  FindPackageShare("flexiv_amr_nav2"),
                                  "maps", "supermarket.yaml",
                              ]),
                              description="Map yaml to localize against "
                                          "(localization_backend:=amcl)"),
        DeclareLaunchArgument("use_rviz", default_value="false",
                              description="Launch RViz2"),
        DeclareLaunchArgument("mock_arms", default_value="false",
                              description="Use mock hardware for the arms "
                                          "(no real robot connection)"),
        DeclareLaunchArgument("enable_waist_driver", default_value="false",
                              description="Publish real AGV_Joint1/2 from the RDK "
                                          "stream (races joint_state_merger's zeros; "
                                          "see aico2_waist_driver/README.md)"),
        DeclareLaunchArgument("direct_joint_states", default_value="false",
                              description="true: arms publish straight to "
                                          "/joint_states and joint_state_merger is "
                                          "retired (see arms.launch.py)"),

        # ============================================================
        # Arms: both Rizon lifecycle drivers, optional waist, merger
        # ============================================================
        _include("flexiv_amr_bringup", "arms.launch.py", {
            "mock_hardware": mock_arms,
            "enable_waist_driver": LaunchConfiguration("enable_waist_driver"),
            "direct_joint_states": LaunchConfiguration("direct_joint_states"),
        }),

        # ============================================================
        # Hardware: URDF/TF + chassis (Robokit) + chassis sensors +
        # both cameras + depth-to-laserscan + scan merger
        # ============================================================
        _include("flexiv_amr_bringup", "hardware_test.launch.py", {
            "use_robokit": "true",
            "use_camera": use_camera,
            "use_top_camera": use_camera,
            "use_visual_odom": use_visual_odom,
            "laserscan_topics": LASERSCAN_TOPICS,
            "merger_delay": str(MERGER_DELAY),
        }),

        # ============================================================
        # Sensor fusion
        # ============================================================
        TimerAction(
            period=EKF_DELAY,
            actions=[_include("flexiv_amr_nav2", "ekf.launch.py")],
        ),

        # ============================================================
        # Mapping: SLAM Toolbox, or RTAB-Map
        # ============================================================
        TimerAction(
            period=SLAM_DELAY,
            actions=[
                _include("flexiv_amr_nav2", "slam.launch.py",
                         condition=IfCondition(use_slam)),
            ],
        ),
        TimerAction(
            period=RTABMAP_MAP_DELAY,
            actions=[
                _include("flexiv_amr_nav2", "rtabmap_mapping.launch.py",
                         condition=IfCondition(use_rtabmap)),
            ],
        ),

        # ============================================================
        # Localization (use_nav only) — exactly one of these publishes
        # map -> odom + /map. Default is RTAB-Map against rtabmap_db
        # (lidar + visual features); localization_backend:=amcl switches
        # to nav2's own AMCL (laser-only, needs map:=).
        # ============================================================
        TimerAction(
            period=LOCALIZATION_DELAY,
            actions=[
                _include("flexiv_amr_nav2", "rtabmap_localization.launch.py",
                         {"rtabmap_db": rtabmap_db},
                         condition=_nav_backend_is("rtabmap")),
                _include("flexiv_amr_nav2", "localization.launch.py",
                         {"map": map_file},
                         condition=_nav_backend_is("amcl")),
            ],
        ),
        TimerAction(
            period=NAV2_DELAY,
            actions=[
                _include("flexiv_amr_nav2", "navigation.launch.py", {
                    "use_sim_time": "false",
                    "params_file": nav2_params,
                    "autostart": "true",
                }, condition=IfCondition(use_nav)),
            ],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(use_rviz),
        ),
    ])
