"""move_group for a single Rizon 10s arm.

The arm driver (rizon10s_driver) serves the FollowJointTrajectory action
move_group executes on. robot_state_publisher must be running to provide TF.

Typical launch order:
    ros2 launch rizon10s_driver arm_driver.launch.py
    ros2 launch rizon10s_description display.launch.py use_gui:=false
    ros2 launch rizon10s_moveit_config move_group.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

_PKG = "rizon10s_moveit_config"


def generate_launch_description():
    log_level = LaunchConfiguration("log_level")

    xacro_file = os.path.join(
        get_package_share_directory("rizon10s_description"),
        "urdf",
        "rizon10s.urdf.xacro",
    )
    robot_description = Command(["xacro ", xacro_file])

    moveit_config = (
        MoveItConfigsBuilder("Rizon10s", package_name=_PKG)
        .robot_description(file_path=xacro_file)
        .robot_description_semantic(file_path="config/rizon10s.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="info"),
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
    ])
