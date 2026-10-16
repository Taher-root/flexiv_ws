"""Reuse the moveit-teleop full-IK/XR stack with the AICO2 config.

Selects left or right arm configuration via the 'arm' argument.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def _resolve_and_include(context):
    arm = LaunchConfiguration('arm').perform(context)
    config_name = 'teleop.yaml' if arm == 'left' else 'right_teleop.yaml'
    config_path = os.path.join(
        get_package_share_directory('aico2_teleop'), 'config', config_name)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('moveit_teleop_bringup'),
                                       '/launch/teleop.launch.py']),
        launch_arguments={
            'robot_config': config_path,
            'joint_states_topic': '/joint_states',
            'goto_ready': LaunchConfiguration('goto_ready').perform(context),
            'rate_limit_scale': LaunchConfiguration('rate_limit_scale').perform(context),
            'hardware_gate_enabled': LaunchConfiguration('hardware_gate_enabled').perform(context),
            'hand': LaunchConfiguration('hand').perform(context),
        }.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value='left', choices=['left', 'right']),
        DeclareLaunchArgument('goto_ready', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rate_limit_scale', default_value='0.5'),
        DeclareLaunchArgument('hardware_gate_enabled', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('hand', default_value='right',
                              choices=['left', 'right']),
        OpaqueFunction(function=_resolve_and_include),
    ])
