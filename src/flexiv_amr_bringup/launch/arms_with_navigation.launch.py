#!/usr/bin/env python3
"""Both arms plus the Robokit mapping stack.

Arms come up first; the mapping stack follows once they are activated.

Usage:
  ros2 launch flexiv_amr_bringup arms_with_navigation.launch.py
  ros2 launch flexiv_amr_bringup arms_with_navigation.launch.py enable_waist_driver:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

MAPPING_DELAY = 6.0  # wait for the arm lifecycle nodes to finish activating


def _include(package, launch_file, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])
        ),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mock_arms", default_value="false",
                              description="Use mock hardware for the arms"),
        DeclareLaunchArgument("enable_waist_driver", default_value="false",
                              description="Publish real AGV_Joint1/2 from the RDK "
                                          "stream (see aico2_waist_driver/README.md)"),
        DeclareLaunchArgument("direct_joint_states", default_value="false",
                              description="true: arms publish straight to "
                                          "/joint_states and joint_state_merger is "
                                          "retired (see arms.launch.py)"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Launch RViz2"),

        _include("flexiv_amr_bringup", "arms.launch.py", {
            "mock_hardware": LaunchConfiguration("mock_arms"),
            "enable_waist_driver": LaunchConfiguration("enable_waist_driver"),
            "direct_joint_states": LaunchConfiguration("direct_joint_states"),
        }),

        TimerAction(
            period=MAPPING_DELAY,
            actions=[
                _include("flexiv_amr_bringup", "mapping_robokit.launch.py",
                         {"use_rviz": LaunchConfiguration("use_rviz")}),
            ],
        ),
    ])
