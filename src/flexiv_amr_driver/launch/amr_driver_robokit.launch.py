#!/usr/bin/env python3
"""AMR chassis with the Robokit backend.

Thin alias for `amr_driver.launch.py use_robokit:=true`, kept because this
filename is what the READMEs and the bringup launches call.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("flexiv_amr_driver"),
                    "launch",
                    "amr_driver.launch.py",
                ])
            ),
            launch_arguments={"use_robokit": "true"}.items(),
        ),
    ])
