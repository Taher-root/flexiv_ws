"""RViz with the MoveIt Motion Planning panel.

Needs move_group.launch.py and the drivers already running: RViz reads the
planning scene move_group publishes and the TF robot_state_publisher provides.
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
    moveit_config = (
        MoveItConfigsBuilder("AICO2", package_name=_PKG)
        .robot_description(file_path=_URDF)
        .robot_description_semantic(file_path="config/aico2.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    rviz_config = os.path.join(
        get_package_share_directory(_PKG), "config", "moveit.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("rviz_config", default_value=rviz_config),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        ),
    ])
