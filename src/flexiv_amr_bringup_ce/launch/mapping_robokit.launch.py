#!/usr/bin/env python3
"""Mapping with the Robokit chassis backend.

AICO 2-4CE variant.
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
                    FindPackageShare("flexiv_amr_bringup_ce"),
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
