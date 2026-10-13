"""MoveIt Servo for the Rizon 10s arm.

Servo takes Cartesian twist commands (TwistStamped) and publishes
JointTrajectory to rt_bridge's servo_joint_command topic, with online
collision checking and singularity management.

Requires move_group running and /arm/joint_states publishing.
Plan to the `ready` named state before engaging teleop.

Usage:
    ros2 launch rizon10s_moveit_config servo.launch.py
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

_PKG = "rizon10s_moveit_config"


def _load_yaml(package_name, file_path):
    full_path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(full_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    log_level = LaunchConfiguration("log_level")

    xacro_file = os.path.join(
        get_package_share_directory("rizon10s_description"),
        "urdf",
        "rizon10s.urdf.xacro",
    )

    moveit_config = (
        MoveItConfigsBuilder("Rizon10s", package_name=_PKG)
        .robot_description(file_path=xacro_file)
        .robot_description_semantic(file_path="config/rizon10s.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    servo_yaml = _load_yaml(_PKG, "config/servo_params.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="info"),
        Node(
            package="moveit_servo",
            executable="servo_node",
            name="servo_node",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                moveit_config.to_dict(),
                servo_params,
                {"use_sim_time": False},
            ],
        ),
    ])
