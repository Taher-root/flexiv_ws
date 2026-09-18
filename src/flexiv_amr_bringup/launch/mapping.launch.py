#!/usr/bin/env python3
"""Mapping: hardware + EKF + SLAM Toolbox.

Usage:
  ros2 launch flexiv_amr_bringup mapping.launch.py
  ros2 launch flexiv_amr_bringup mapping.launch.py use_robokit:=true
  ros2 launch flexiv_amr_bringup mapping.launch.py use_rviz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(package, launch_file, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])
        ),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_robokit", default_value="false",
                              description="Use the Robokit chassis velocity controller"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Launch RViz2"),

        # Hardware layer: URDF/TF + chassis + sensors
        _include("flexiv_amr_bringup", "hardware_test.launch.py",
                 {"use_robokit": LaunchConfiguration("use_robokit")}),

        # Sensor fusion (publishes odom -> base_link)
        _include("flexiv_amr_nav2", "ekf.launch.py"),

        # SLAM Toolbox (mapping from /scan/merged)
        _include("flexiv_amr_nav2", "slam.launch.py"),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
