"""MoveIt Servo for the left Rizon 4 arm.

Servo runs as a standalone node alongside move_group. It subscribes to
TwistStamped (Cartesian velocity from VR IK or a joystick) and publishes
JointTrajectory to rt_bridge's servo_joint_command topic, with online
collision checking and singularity management.

Requires move_group running (for the planning scene) and /left_arm/joint_states
publishing (from rt_bridge or the NRT arm driver). Plan to the `ready` named
state before engaging -- home has joint 6 at 0 (wrist singularity).

Usage:
  ros2 launch aico2_moveit_config servo.launch.py

  # From another terminal, send a twist:
  ros2 topic pub --once /servo_node/delta_twist_cmds \\
      geometry_msgs/TwistStamped \\
      '{header: {frame_id: Left_link0}, twist: {linear: {z: 0.3}}}'
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

_PKG = "aico2_moveit_config"
_URDF = os.path.join(
    get_package_share_directory("flexiv_amr_description"),
    "urdf",
    "AICO2-Rizon4.urdf",
)


def generate_launch_description():
    log_level = LaunchConfiguration("log_level")

    moveit_config = (
        MoveItConfigsBuilder("AICO2", package_name=_PKG)
        .robot_description(file_path=_URDF)
        .robot_description_semantic(file_path="config/aico2.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    servo_params_path = os.path.join(
        get_package_share_directory(_PKG), "config", "servo_params.yaml"
    )

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
                servo_params_path,
                {"use_sim_time": False},
            ],
        ),
    ])
