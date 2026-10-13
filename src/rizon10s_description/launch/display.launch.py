"""Visualise the Rizon 10s URDF in RViz with joint_state_publisher_gui.

Usage:
    ros2 launch rizon10s_description display.launch.py
    ros2 launch rizon10s_description display.launch.py use_gui:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gui = LaunchConfiguration("use_gui")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("flexiv_description"), "urdf", "flexiv.urdf.xacro"]
    )
    robot_description = Command(["xacro ", xacro_file, " robot_type:=Rizon10s"])

    rviz_config = os.path.join(
        get_package_share_directory("rizon10s_description"),
        "launch",
        "display.rviz",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_gui", default_value="true",
            description="Launch joint_state_publisher_gui for interactive sliders",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(use_gui),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=UnlessCondition(use_gui),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_gui),
        ),
    ])
