#!/usr/bin/env python3
"""Mapping with the Robokit chassis backend.

Thin alias for `mapping.launch.py use_robokit:=true`, kept because this
filename is what the READMEs and arms_with_navigation.launch.py call.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Launch RViz2"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("flexiv_amr_bringup"),
                    "launch",
                    "mapping.launch.py",
                ])
            ),
            launch_arguments={
                "use_robokit": "true",
                "use_rviz": LaunchConfiguration("use_rviz"),
            }.items(),
        ),
    ])
