"""VR teleoperation for the AICO2 dual-arm system.

Composes arm drivers, robot state publisher, MoveIt move_group, trajectory
adapters, the moveit-teleop IK pipeline, and a joystick relay into a single
launch.

Both arms use symmetric 7-DOF IK chains (left_arm / right_arm).  The waist
(AGV_Joint1/2) and AMR chassis are controlled independently via the Quest
thumbsticks: right stick -> waist velocity, left stick -> AMR /cmd_vel.
The left arm driver still owns the waist axes (control_waist:=true) so the
joystick relay can override them through the waist_override topic.

Usage:
    # Mock hardware, left arm (default):
    ros2 launch aico2_teleop vr.launch.py

    # Real hardware, left arm (NRT Python driver):
    ros2 launch aico2_teleop vr.launch.py hardware:=true

    # Real hardware, RT backend (needs rt_server running in a noros.sh shell):
    ros2 launch aico2_teleop vr.launch.py hardware:=true hardware_backend:=rt

    # Mock hardware, right arm only:
    ros2 launch aico2_teleop vr.launch.py arm:=right

    # Both arms — bimanual VR.  Left Quest controller drives the left arm,
    # right controller drives the right.  Two XR bridge tabs required on the
    # headset: https://<ip>:8443 (left) and https://<ip>:8444 (right).
    ros2 launch aico2_teleop vr.launch.py arm:=both

    # Slow rate limit for initial hardware testing:
    ros2 launch aico2_teleop vr.launch.py hardware:=true rate_limit_scale:=0.1
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, Shutdown, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

LEFT_JOINTS = [f'Left_joint{i}' for i in range(1, 8)]
RIGHT_JOINTS = [f'Right_joint{i}' for i in range(1, 8)]

_MOVEIT_PKG = 'aico2_moveit_config'
_URDF = os.path.join(
    get_package_share_directory('flexiv_amr_description_ce'),
    'urdf', 'AICO2-4CE-Rizon4.urdf',
)

_XR_PORT = {'left': 8443, 'right': 8444}

_RT_SHM = {
    'left':  '/aico2_rt_left',
    'right': '/aico2_rt_right',
}

_RT_JOINT_NAMES = {
    'left':  ['AGV_Joint1', 'AGV_Joint2'] + [f'Left_joint{i}' for i in range(1, 8)],
    'right': [f'Right_joint{i}' for i in range(1, 8)],
}

_RT_SERVO_JOINT_NAMES = {
    'left':  [f'Left_joint{i}' for i in range(1, 8)],
    'right': [f'Right_joint{i}' for i in range(1, 8)],
}


def _include(package, filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare(package), '/launch/', filename]),
        launch_arguments=(arguments or {}).items())


def _teleop_pipeline(arm, hardware, rate_limit_scale, goto_ready,
                     *, input_topic=None, device='xr', hand=None):
    """Include the moveit-teleop pipeline for one arm."""
    config_name = 'teleop.yaml' if arm == 'left' else 'right_teleop.yaml'
    config_path = os.path.join(
        get_package_share_directory('aico2_teleop'), 'config', config_name)
    args = {
        'robot_config': config_path,
        'joint_states_topic': '/joint_states',
        'goto_ready': goto_ready,
        'rate_limit_scale': rate_limit_scale,
        'hardware_gate_enabled': str(hardware).lower(),
        'hand': hand or ('left' if arm == 'left' else 'right'),
        'device': device,
    }
    if input_topic:
        args['input_topic'] = input_topic
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('moveit_teleop_bringup'),
                                       '/launch/teleop.launch.py']),
        launch_arguments=args.items())


def _rt_bridge_nodes(arms_to_launch):
    """Launch rt_bridge nodes (one per arm) + joint_state_merger."""
    nodes = []
    for arm in arms_to_launch:
        nodes.append(Node(
            package='aico2_rt_control',
            executable='rt_bridge',
            name='rt_bridge',
            namespace=f'{arm}_arm',
            output='screen',
            on_exit=Shutdown(),
            remappings=[('joint_states', 'joint_states')],
            parameters=[{
                'shm_name':           _RT_SHM[arm],
                'joint_names':        _RT_JOINT_NAMES[arm],
                'servo_joint_names':  _RT_SERVO_JOINT_NAMES[arm],
                'joint_state_rate_hz': 200.0,
            }],
        ))

    nodes.append(Node(
        package='flexiv_amr_driver',
        executable='joint_state_merger',
        name='joint_state_merger',
        output='screen',
        parameters=[{
            'left_arm_topic':  '/left_arm/joint_states',
            'right_arm_topic': '/right_arm/joint_states',
            'rate_hz': 50.0,
        }],
    ))
    return nodes


def _build(context):
    arm = LaunchConfiguration('arm').perform(context)
    hardware = LaunchConfiguration('hardware').perform(context) == 'true'
    mock = not hardware
    rate_limit_scale = LaunchConfiguration('rate_limit_scale').perform(context)
    backend = LaunchConfiguration('hardware_backend').perform(context)
    bimanual = arm == 'both'
    rt = (backend == 'rt') and hardware

    actions = []

    # 1. Arm drivers + joint state merger.
    if rt:
        arms_to_launch = ['left', 'right'] if bimanual else [arm]
        actions.extend(_rt_bridge_nodes(arms_to_launch))
    else:
        use_waist = arm in ('left', 'both')
        actions.append(_include('flexiv_amr_bringup_ce', 'arms.launch.py', {
            'mock_hardware': 'true' if mock else 'false',
            'teleop_backend': 'servo',
            'control_waist': 'true' if use_waist else 'false',
        }))

    # 2. Robot state publisher.
    actions.append(_include('flexiv_amr_description_ce', 'display.launch.py', {
        'use_gui': 'false',
    }))

    # 3. MoveIt move_group.
    actions.append(_include('aico2_moveit_config', 'move_group.launch.py'))

    # 4. Trajectory adapters.
    moveit_config = (
        MoveItConfigsBuilder('AICO2', package_name=_MOVEIT_PKG)
        .robot_description(file_path=_URDF)
        .to_moveit_configs()
    )

    arms_for_teleop = ['left', 'right'] if bimanual else [arm]

    for a in arms_for_teleop:
        joints = LEFT_JOINTS if a == 'left' else RIGHT_JOINTS
        adapter_params = {'joint_names': joints}
        if bimanual:
            adapter_params['input_topic'] = f'/{a}_teleop/xr_device/input'
        actions.append(Node(
            package='aico2_teleop', executable='trajectory_adapter.py',
            name='trajectory_adapter', namespace=f'{a}_arm', output='screen',
            parameters=[moveit_config.robot_description, adapter_params]))

    # 5. Teleop pipelines.
    requested = LaunchConfiguration('goto_ready').perform(context)
    if rt:
        home = 'false'
    else:
        home = requested if requested else ('false' if hardware else 'true')

    teleop_delay = 5.0 if rt else 3.0

    teleop_actions = []
    if bimanual:
        for a in ('left', 'right'):
            xr_ns = f'{a}_teleop'
            actions.append(_include('moveit_teleop_bringup', 'xr_device.launch.py', {
                'hand': a,
                'port': str(_XR_PORT[a]),
                'namespace': xr_ns,
            }))
            teleop_actions.append(_teleop_pipeline(
                a, hardware, rate_limit_scale, home,
                device='fake',
                input_topic=f'/{xr_ns}/xr_device/input',
                hand=a))
    else:
        teleop_actions.append(
            _teleop_pipeline(arm, hardware, rate_limit_scale, home))

    actions.append(TimerAction(period=teleop_delay, actions=teleop_actions))

    # 6. Joystick relay: right stick -> waist, left stick -> AMR
    joy_topic = (f'/left_teleop/xr_device/thumbsticks' if bimanual
                 else '/xr_device/thumbsticks')
    actions.append(Node(
        package='aico2_teleop', executable='vr_joystick_relay.py',
        name='vr_joystick_relay', output='screen',
        parameters=[{'joy_topic': joy_topic}],
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value='left',
                              choices=['left', 'right', 'both']),
        DeclareLaunchArgument('hardware', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('hardware_backend', default_value='nrt',
                              choices=['nrt', 'rt'],
                              description='nrt: Python arm drivers. '
                                          'rt: rt_bridge over shared memory '
                                          '(needs rt_server in a noros.sh shell).'),
        DeclareLaunchArgument('rate_limit_scale', default_value='0.5'),
        DeclareLaunchArgument('goto_ready', default_value='',
                              choices=['', 'true', 'false']),
        OpaqueFunction(function=_build),
    ])
