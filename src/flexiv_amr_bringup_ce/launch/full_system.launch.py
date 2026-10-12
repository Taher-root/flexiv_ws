#!/usr/bin/env python3
"""Complete AICO 2-4CE system: arms + chassis + sensors + mapping or navigation.

AICO 2-4CE variant (software v3.11.2 CE, RDK v1.9.3).

Composed from the per-subsystem launch files rather than restating them:

  arms.launch.py                  (flexiv_amr_bringup_ce)  both Rizon arms, waist, merger
  hardware_test.launch.py         (flexiv_amr_bringup_ce)  URDF/TF + chassis + sensors
    display.launch.py             (flexiv_amr_description_ce)
    amr_driver.launch.py          (flexiv_amr_driver)
    sensors.launch.py             (flexiv_amr_sensors)
  ekf.launch.py                   (flexiv_amr_nav2)  sensor fusion
  slam.launch.py                  (flexiv_amr_nav2)  SLAM Toolbox mapping    [use_slam]
  rtabmap_mapping.launch.py       (flexiv_amr_nav2)  RTAB-Map mapping        [use_rtabmap]
  rtabmap_localization.launch.py  (flexiv_amr_nav2)  RTAB-Map localization  [use_nav, default]
  localization.launch.py          (flexiv_amr_nav2)  AMCL + map_server      [backend=amcl]
  move_group.launch.py            (aico2_moveit_config)  arm motion planning   [use_moveit]
  navigation.launch.py            (flexiv_amr_nav2)  Nav2 stack              [use_nav]

Usage:
  # Mapping with slam_toolbox (default)
  ros2 launch flexiv_amr_bringup_ce full_system.launch.py

  # Navigation
  ros2 launch flexiv_amr_bringup_ce full_system.launch.py use_slam:=false use_nav:=true

  # With MoveIt planning for the arms
  ros2 launch flexiv_amr_bringup_ce full_system.launch.py use_moveit:=true

  # Mock arms (no robot connection)
  ros2 launch flexiv_amr_bringup_ce full_system.launch.py mock_arms:=true
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

LASERSCAN_TOPICS = "/scan/nav /scan/avoid /scan/depth"
TOP_LASERSCAN_TOPIC = "/scan/top"

MERGER_DELAY = 3.0
EKF_DELAY = 4.0
SLAM_DELAY = 8.0
RTABMAP_MAP_DELAY = 15.0
LOCALIZATION_DELAY = 10.0
NAV2_DELAY = 20.0
MOVEIT_DELAY = 6.0


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
    use_top_camera = LaunchConfiguration("use_top_camera")
    laserscan_topics = PythonExpression([
        "'", LASERSCAN_TOPICS, " ", TOP_LASERSCAN_TOPIC, "' if '",
        use_top_camera, "' == 'true' else '", LASERSCAN_TOPICS, "'",
    ])
    use_slam = LaunchConfiguration("use_slam")
    use_rtabmap = LaunchConfiguration("use_rtabmap")
    use_nav = LaunchConfiguration("use_nav")
    use_rviz = LaunchConfiguration("use_rviz")
    map_file = LaunchConfiguration("map")
    rtabmap_db = LaunchConfiguration("rtabmap_db")
    localization_backend = LaunchConfiguration("localization_backend")
    mock_arms = LaunchConfiguration("mock_arms")
    use_moveit = LaunchConfiguration("use_moveit")

    nav2_params = PathJoinSubstitution(
        [FindPackageShare("flexiv_amr_nav2"), "config", "nav2_params.yaml"]
    )

    def _nav_backend_is(backend):
        return IfCondition(PythonExpression(
            ["'", use_nav, "' == 'true' and '", localization_backend, f"' == '{backend}'"]
        ))

    return LaunchDescription([
        DeclareLaunchArgument("use_camera", default_value="true",
                              description="Launch both RealSense cameras + depth-to-laserscan"),
        DeclareLaunchArgument("use_robokit", default_value="true",
                              description="Use the Robokit chassis velocity controller"),
        DeclareLaunchArgument("use_top_camera", default_value="false",
                              description="Also bring up the head-mounted top D456"),
        DeclareLaunchArgument("use_visual_odom", default_value="true",
                              description="Launch RTAB-Map visual odometry"),
        DeclareLaunchArgument("use_slam", default_value="true",
                              description="Launch SLAM Toolbox for mapping"),
        DeclareLaunchArgument("use_rtabmap", default_value="false",
                              description="Launch RTAB-Map instead of slam_toolbox"),
        DeclareLaunchArgument("use_nav", default_value="false",
                              description="Launch localization + Nav2 stack"),
        DeclareLaunchArgument("localization_backend", default_value="rtabmap",
                              description="'rtabmap' or 'amcl'"),
        DeclareLaunchArgument("rtabmap_db",
                              default_value=PathJoinSubstitution([
                                  FindPackageShare("flexiv_amr_nav2"),
                                  "maps", "rtabmap_backup.db",
                              ]),
                              description="RTAB-Map database to localize against"),
        DeclareLaunchArgument("map",
                              default_value=PathJoinSubstitution([
                                  FindPackageShare("flexiv_amr_nav2"),
                                  "maps", "supermarket.yaml",
                              ]),
                              description="Map yaml to localize against (amcl)"),
        DeclareLaunchArgument("joint_control_mode", default_value="position",
                              description="'position' or 'impedance'"),
        DeclareLaunchArgument("joint_stiffness_ratio", default_value="1.0",
                              description="fraction of K_q_nom in impedance mode"),
        DeclareLaunchArgument("max_contact_torque", default_value="0.0",
                              description="Nm ceiling per axis in impedance mode"),
        DeclareLaunchArgument("use_moveit", default_value="false",
                              description="Launch move_group for arm motion planning"),
        DeclareLaunchArgument("use_rviz", default_value="false",
                              description="Launch RViz2"),
        DeclareLaunchArgument("mock_arms", default_value="false",
                              description="Use mock hardware for the arms"),
        DeclareLaunchArgument("enable_waist_driver", default_value="false",
                              description="Publish real AGV_Joint1/2 from RDK stream"),

        # Arms
        _include("flexiv_amr_bringup_ce", "arms.launch.py", {
            "mock_hardware": mock_arms,
            "enable_waist_driver": LaunchConfiguration("enable_waist_driver"),
            "joint_control_mode": LaunchConfiguration("joint_control_mode"),
            "joint_stiffness_ratio": LaunchConfiguration("joint_stiffness_ratio"),
            "max_contact_torque": LaunchConfiguration("max_contact_torque"),
        }),

        # Hardware: URDF/TF + chassis + sensors
        _include("flexiv_amr_bringup_ce", "hardware_test.launch.py", {
            "use_robokit": LaunchConfiguration("use_robokit"),
            "use_camera": use_camera,
            "use_top_camera": use_top_camera,
            "use_visual_odom": use_visual_odom,
            "laserscan_topics": laserscan_topics,
            "merger_delay": str(MERGER_DELAY),
        }),

        # Sensor fusion
        TimerAction(
            period=EKF_DELAY,
            actions=[_include("flexiv_amr_nav2", "ekf.launch.py")],
        ),

        # Mapping
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

        # Localization
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

        # Arm motion planning
        TimerAction(
            period=MOVEIT_DELAY,
            actions=[
                _include("aico2_moveit_config", "move_group.launch.py",
                         condition=IfCondition(use_moveit)),
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
