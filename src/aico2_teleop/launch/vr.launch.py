"""VR teleoperation for the AICO2 dual-arm system.

Composes arm drivers, robot state publisher, MoveIt move_group, trajectory
adapters, and the moveit-teleop IK pipeline into a single launch.

Usage:
    # Mock hardware, left arm (default):
    ros2 launch aico2_teleop vr.launch.py

    # Real hardware, left arm:
    ros2 launch aico2_teleop vr.launch.py hardware:=true

    # Mock hardware, right arm:
    ros2 launch aico2_teleop vr.launch.py arm:=right

    # Slow rate limit for initial hardware testing:
    ros2 launch aico2_teleop vr.launch.py hardware:=true rate_limit_scale:=0.1
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

LEFT_JOINTS = [f'Left_joint{i}' for i in range(1, 8)]
RIGHT_JOINTS = [f'Right_joint{i}' for i in range(1, 8)]

_MOVEIT_PKG = 'aico2_moveit_config'
_URDF = os.path.join(
    get_package_share_directory('flexiv_amr_description'),
    'urdf', 'AICO2-Rizon4.urdf',
)


def _include(package, filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare(package), '/launch/', filename]),
        launch_arguments=(arguments or {}).items())


def _build(context):
    arm = LaunchConfiguration('arm').perform(context)
    hardware = LaunchConfiguration('hardware').perform(context) == 'true'
    mock = not hardware

    actions = []

    # 1. Arm drivers + joint state merger.
    #    teleop_backend:=servo so both drivers accept servo_joint_command.
    actions.append(_include('flexiv_amr_bringup', 'arms.launch.py', {
        'mock_hardware': 'true' if mock else 'false',
        'teleop_backend': 'servo',
    }))

    # 2. Robot state publisher (TF from /joint_states).
    actions.append(_include('flexiv_amr_description', 'display.launch.py', {
        'use_gui': 'false',
    }))

    # 3. MoveIt move_group (planning, collision checking).
    actions.append(_include('aico2_moveit_config', 'move_group.launch.py'))

    # 4. Trajectory adapter for the arm being teleoperated.
    #    Runs in the arm's namespace so topics resolve to /<ns>/....
    moveit_config = (
        MoveItConfigsBuilder('AICO2', package_name=_MOVEIT_PKG)
        .robot_description(file_path=_URDF)
        .to_moveit_configs()
    )

    joints = LEFT_JOINTS if arm == 'left' else RIGHT_JOINTS
    actions.append(Node(
        package='aico2_teleop', executable='trajectory_adapter.py',
        name='trajectory_adapter', namespace=f'{arm}_arm', output='screen',
        parameters=[moveit_config.robot_description, {
            'joint_names': joints,
        }]))

    # 5. The moveit-teleop pipeline (IK solver, retargeting, XR device, etc.).
    #    Delayed so move_group and the drivers have time to settle.
    requested = LaunchConfiguration('goto_ready').perform(context)
    home = requested if requested else ('false' if hardware else 'true')
    teleop = _include('aico2_teleop', 'teleop.launch.py', {
        'arm': arm,
        'goto_ready': home,
        'rate_limit_scale': LaunchConfiguration('rate_limit_scale').perform(context),
        'hardware_gate_enabled': str(hardware).lower(),
    })
    actions.append(TimerAction(period=3.0, actions=[teleop]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value='left',
                              choices=['left', 'right']),
        DeclareLaunchArgument('hardware', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('rate_limit_scale', default_value='0.5'),
        DeclareLaunchArgument('goto_ready', default_value='',
                              choices=['', 'true', 'false']),
        OpaqueFunction(function=_build),
    ])
