#!/usr/bin/env python3
"""Complete AICO2 system: arms + chassis + sensors + mapping or navigation.

Composed from the per-subsystem launch files rather than restating them:

  arms.launch.py            (flexiv_amr_bringup)  both Rizon arms, waist, merger
  hardware_test.launch.py   (flexiv_amr_bringup)  URDF/TF + chassis + sensors
    display.launch.py       (flexiv_amr_description)
    amr_driver.launch.py    (flexiv_amr_driver)
    sensors.launch.py       (flexiv_amr_sensors)
  ekf.launch.py             (flexiv_amr_nav2)     sensor fusion
  slam.launch.py            (flexiv_amr_nav2)     SLAM Toolbox      [use_slam]
  rtabmap_mapping.launch.py (flexiv_amr_nav2)     RTAB-Map SLAM     [use_rtabmap]
  localization.launch.py    (nav2_bringup)        AMCL + map_server [use_nav]
  navigation.launch.py      (flexiv_amr_nav2)     Nav2 stack        [use_nav]

The staged start delays live here, because they are a property of bringing the
whole system up at once, not of the subsystems themselves.

Usage:
  # Mapping with slam_toolbox (default)
  ros2 launch flexiv_amr_bringup full_system.launch.py

  # Mapping with RTAB-Map
  ros2 launch flexiv_amr_bringup full_system.launch.py use_slam:=false use_rtabmap:=true

  # Navigation against a saved map
  ros2 launch flexiv_amr_bringup full_system.launch.py use_slam:=false use_nav:=true
  ros2 launch flexiv_amr_bringup full_system.launch.py use_slam:=false \
      use_nav:=true map:=/path/to/map.yaml

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
MERGER_DELAY = 3.0    # TF tree + scan sources up
EKF_DELAY = 4.0       # odom + IMU sources publishing
SLAM_DELAY = 8.0      # arms + EKF + merged scan settled
RTABMAP_DELAY = 15.0  # both cameras enumerated on USB and streaming
AMCL_DELAY = 10.0
NAV2_DELAY = 20.0     # map_server + AMCL active, /map and map->odom TF up


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
    mock_arms = LaunchConfiguration("mock_arms")

    nav2_params = PathJoinSubstitution(
        [FindPackageShare("flexiv_amr_nav2"), "config", "nav2_params.yaml"]
    )

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
                              description="Launch Nav2 (AMCL + map_server + navigation stack)"),
        DeclareLaunchArgument("map",
                              default_value=PathJoinSubstitution([
                                  FindPackageShare("flexiv_amr_nav2"),
                                  "maps", "supermarket.yaml",
                              ]),
                              description="Map yaml for navigation mode"),
        DeclareLaunchArgument("use_rviz", default_value="false",
                              description="Launch RViz2"),
        DeclareLaunchArgument("mock_arms", default_value="false",
                              description="Use mock hardware for the arms "
                                          "(no real robot connection)"),
        DeclareLaunchArgument("enable_waist_driver", default_value="false",
                              description="Publish real AGV_Jiont1/2 from the RDK "
                                          "stream (races joint_state_merger's zeros; "
                                          "see aico2_waist_driver/README.md)"),

        # ============================================================
        # Arms: both Rizon lifecycle drivers, optional waist, merger
        # ============================================================
        _include("flexiv_amr_bringup", "arms.launch.py", {
            "mock_hardware": mock_arms,
            "enable_waist_driver": LaunchConfiguration("enable_waist_driver"),
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
            period=RTABMAP_DELAY,
            actions=[
                _include("flexiv_amr_nav2", "rtabmap_mapping.launch.py",
                         condition=IfCondition(use_rtabmap)),
            ],
        ),

        # ============================================================
        # Navigation. nav2_bringup's own localization_launch.py is used
        # for AMCL + map_server, but its bringup_launch.py is not: the
        # Jazzy version pulls in route_server and docking_server, which
        # we do not need and which block lifecycle activation.
        # ============================================================
        TimerAction(
            period=AMCL_DELAY,
            actions=[
                _include("nav2_bringup", "localization_launch.py", {
                    "map": map_file,
                    "use_sim_time": "false",
                    "params_file": nav2_params,
                    "autostart": "true",
                    "use_composition": "False",
                }, condition=IfCondition(PythonExpression(
                    ["'", use_nav, "' == 'true' and '", use_rtabmap, "' != 'true'"]
                ))),
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
