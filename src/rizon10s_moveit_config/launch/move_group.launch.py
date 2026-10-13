"""move_group + robot_state_publisher + RViz for a single Rizon 10s arm.

Launches everything needed for MoveIt Plan & Execute through the RT bridge.
rt_bridge (and rt_server behind it) must be running separately.

Usage:
    # RT pipeline (3 terminals):
    #   T1 (noros): sudo ./standalone/build/rt_server Rizon10s-062545
    #   T2:        ros2 run rizon10s_rt_control rt_bridge --ros-args -r __ns:=/arm
    #   T3:        ros2 launch rizon10s_moveit_config move_group.launch.py

    # Without RViz (headless):
    ros2 launch rizon10s_moveit_config move_group.launch.py rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

_PKG = "rizon10s_moveit_config"


def generate_launch_description():
    log_level = LaunchConfiguration("log_level")
    use_rviz = LaunchConfiguration("rviz")

    xacro_file = os.path.join(
        get_package_share_directory("flexiv_description"),
        "urdf",
        "flexiv.urdf.xacro",
    )
    xacro_mappings = {"robot_type": "Rizon10s"}
    robot_description = Command(
        ["xacro ", xacro_file, " robot_type:=Rizon10s"]
    )

    moveit_config = (
        MoveItConfigsBuilder("Rizon10s", package_name=_PKG)
        .robot_description(file_path=xacro_file, mappings=xacro_mappings)
        .robot_description_semantic(file_path="config/rizon10s.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    rviz_config = os.path.join(
        get_package_share_directory(_PKG), "config", "moveit.rviz"
    )

    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Launch RViz with MoveIt panel"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),

        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                moveit_config.to_dict(),
                {
                    "publish_robot_description": True,
                    "publish_robot_description_semantic": True,
                    "publish_planning_scene": True,
                    "publish_geometry_updates": True,
                    "publish_state_updates": True,
                    "publish_transforms_updates": True,
                    "use_sim_time": False,
                },
            ],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
            condition=IfCondition(use_rviz),
        ),
    ])
