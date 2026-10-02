#!/usr/bin/env python3
"""AMR chassis: velocity control + wheel odometry + status monitor.

Two backends, selected with use_robokit. Prefer true: the flexivamr SDK does
not support continuous velocity control, so the raw-TCP path is what Nav2 is
driven through. velocity_controller is kept for its gain-control handshake,
not as an equal alternative.

  true             robokit_velocity_controller + config/amr_params_robokit.yaml
                   Seer API 2010 on port 19205 at 20 Hz, no vendor SDK.
                   publish_tf off so EKF owns odom->base_link.
  false (default)  velocity_controller        + config/amr_params.yaml
                   Requires the flexivamr SDK. Without it this node starts,
                   fails to connect, and silently drops every /cmd_vel.

NOTE: this default disagrees with full_system.launch.py, which defaults
use_robokit to true. Pass it explicitly rather than relying on either.

Usage:
  ros2 launch flexiv_amr_driver amr_driver.launch.py
  ros2 launch flexiv_amr_driver amr_driver.launch.py use_robokit:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_robokit = LaunchConfiguration("use_robokit")

    # One params file, chosen by the same flag that chooses the controller.
    params_file = PathJoinSubstitution([
        FindPackageShare("flexiv_amr_driver"),
        "config",
        PythonExpression([
            "'amr_params_robokit.yaml' if '", use_robokit, "' == 'true' "
            "else 'amr_params.yaml'"
        ]),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_robokit",
            default_value="false",
            description="Use the Robokit TCP velocity controller and its params",
        ),

        Node(
            package="flexiv_amr_driver",
            executable="velocity_controller",
            name="velocity_controller",
            output="screen",
            parameters=[params_file],
            condition=UnlessCondition(use_robokit),
        ),
        Node(
            package="flexiv_amr_driver",
            executable="robokit_velocity_controller",
            name="robokit_velocity_controller",
            output="screen",
            parameters=[params_file],
            condition=IfCondition(use_robokit),
        ),

        Node(
            package="flexiv_amr_driver",
            executable="odometry_publisher",
            name="odometry_publisher",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="flexiv_amr_driver",
            executable="status_monitor",
            name="status_monitor",
            output="screen",
            parameters=[params_file],
        ),
    ])
