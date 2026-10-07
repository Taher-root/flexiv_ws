"""MoveIt Servo for a Rizon 4 arm.

Servo runs as a standalone node alongside move_group. It subscribes to
TwistStamped (Cartesian velocity from VR IK or a joystick) and publishes
JointTrajectory to rt_bridge's servo_joint_command topic, with online
collision checking and singularity management.

Requires move_group running (for the planning scene) and /<arm>/joint_states
publishing (from rt_bridge or the NRT arm driver). Plan to the `ready` named
state before engaging -- home has joint 6 at 0 (wrist singularity).

Usage:
  ros2 launch aico2_moveit_config servo.launch.py
  ros2 launch aico2_moveit_config servo.launch.py arm:=right
"""
import os

import yaml
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

_ARM_CONFIG = {
    "left": {
        "move_group_name": "left_arm",
        "planning_frame": "Left_link0",
        "ee_frame_name": "Left_flange",
        "robot_link_command_frame": "Left_link0",
        "command_out_topic": "/left_arm/servo_joint_command",
        "joint_topic": "/left_arm/joint_states",
    },
    "right": {
        "move_group_name": "right_arm",
        "planning_frame": "Right_link0",
        "ee_frame_name": "Right_flange",
        "robot_link_command_frame": "Right_link0",
        "command_out_topic": "/right_arm/servo_joint_command",
        "joint_topic": "/right_arm/joint_states",
    },
}


def _load_yaml(package_name, file_path):
    full_path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(full_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    log_level = LaunchConfiguration("log_level")
    arm_arg = LaunchConfiguration("arm")

    moveit_config = (
        MoveItConfigsBuilder("AICO2", package_name=_PKG)
        .robot_description(file_path=_URDF)
        .robot_description_semantic(file_path="config/aico2.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    servo_yaml = _load_yaml(_PKG, "config/servo_params.yaml")

    # OpaqueFunction resolves the arm arg at launch time
    from launch.actions import OpaqueFunction

    def _launch_servo(context):
        arm = context.launch_configurations.get("arm", "left")
        node_name = context.launch_configurations.get("node_name", "servo_node")
        overrides = _ARM_CONFIG.get(arm, _ARM_CONFIG["left"])
        servo_yaml.update(overrides)
        servo_params = {"moveit_servo": servo_yaml}
        ll = context.launch_configurations.get("log_level", "info")

        return [Node(
            package="moveit_servo",
            executable="servo_node",
            name=node_name,
            output="screen",
            arguments=["--ros-args", "--log-level", ll],
            parameters=[
                moveit_config.to_dict(),
                servo_params,
                {"use_sim_time": False},
            ],
        )]

    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("arm", default_value="left",
                              description="Which arm to servo: left or right"),
        DeclareLaunchArgument("node_name", default_value="servo_node",
                              description="ROS node name for this servo "
                                          "instance (unique per arm)"),
        OpaqueFunction(function=_launch_servo),
    ])
