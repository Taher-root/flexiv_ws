#!/usr/bin/env python3
"""Navigation: hardware + EKF + localization (RTAB-Map or AMCL) + Nav2.

AICO 2-4CE variant.

Usage:
  ros2 launch flexiv_amr_bringup_ce navigation.launch.py
  ros2 launch flexiv_amr_bringup_ce navigation.launch.py rtabmap_db:=/path/to/other.db
  ros2 launch flexiv_amr_bringup_ce navigation.launch.py \
      localization_backend:=amcl map:=/path/to/map.yaml
  ros2 launch flexiv_amr_bringup_ce navigation.launch.py use_robokit:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(package, launch_file, launch_arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])
        ),
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    localization_backend = LaunchConfiguration("localization_backend")

    return LaunchDescription([
        DeclareLaunchArgument(
            "localization_backend", default_value="rtabmap",
            description="'rtabmap' or 'amcl'",
        ),
        DeclareLaunchArgument(
            "rtabmap_db",
            default_value=PathJoinSubstitution([
                FindPackageShare("flexiv_amr_nav2"), "maps", "rtabmap_backup.db"
            ]),
            description="RTAB-Map database to localize against",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=PathJoinSubstitution([
                FindPackageShare("flexiv_amr_nav2"), "maps", "supermarket.yaml"
            ]),
            description="Map yaml to localize against (localization_backend:=amcl)",
        ),
        DeclareLaunchArgument("use_robokit", default_value="false",
                              description="Use the Robokit chassis velocity controller"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Launch RViz2"),

        _include("flexiv_amr_bringup_ce", "hardware_test.launch.py",
                 {"use_robokit": LaunchConfiguration("use_robokit")}),

        _include("flexiv_amr_nav2", "ekf.launch.py"),

        _include("flexiv_amr_nav2", "rtabmap_localization.launch.py",
                 {"rtabmap_db": LaunchConfiguration("rtabmap_db")},
                 condition=IfCondition(PythonExpression(
                     ["'", localization_backend, "' == 'rtabmap'"]
                 ))),
        _include("flexiv_amr_nav2", "localization.launch.py",
                 {"map": LaunchConfiguration("map")},
                 condition=IfCondition(PythonExpression(
                     ["'", localization_backend, "' == 'amcl'"]
                 ))),

        _include("flexiv_amr_nav2", "navigation.launch.py"),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
