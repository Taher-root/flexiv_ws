#!/usr/bin/env python3
"""AMR chassis: velocity control + wheel odometry + status monitor.

Two backends, selected with use_robokit:
  false (default)  velocity_controller        + config/amr_params.yaml
  true             robokit_velocity_controller + config/amr_params_robokit.yaml
                   (smooth velocity control; publish_tf off so EKF owns odom->base_link)

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
