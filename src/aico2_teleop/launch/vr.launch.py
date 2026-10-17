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

    # Both arms — bimanual VR from a SINGLE browser tab.  The Quest headset
    # sends both controllers in every frame; xr_bimanual_bridge publishes
    # each hand to its own TeleopInput topic.
    ros2 launch aico2_teleop vr.launch.py arm:=both

    # Slow rate limit for initial hardware testing:
    ros2 launch aico2_teleop vr.launch.py hardware:=true rate_limit_scale:=0.1
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction,
                            Shutdown, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import xacro

LEFT_JOINTS = [f'Left_joint{i}' for i in range(1, 8)]
RIGHT_JOINTS = [f'Right_joint{i}' for i in range(1, 8)]

_MOVEIT_PKG = 'aico2_moveit_config'

_RT_SHM_DEFAULT = '/aico2_rt_control'
_RT_SHM_BIMANUAL = {
    'left':  '/aico2_rt_left',
    'right': '/aico2_rt_right',
}

_RT_JOINT_NAMES = {
    'left':  ['AGV_Joint1', 'AGV_Joint2'] + [f'Left_joint{i}' for i in range(1, 8)],
    'right': ['AGV_Joint1', 'AGV_Joint2'] + [f'Right_joint{i}' for i in range(1, 8)],
}

_RT_SERVO_JOINT_NAMES = {
    'left':  [f'Left_joint{i}' for i in range(1, 8)],
    'right': [f'Right_joint{i}' for i in range(1, 8)],
}

# flexiv_description xacro produces lowercase prefixes (left_, right_) and its
# own waist names (aico2_4e_d1_*).  The MoveIt config, SRDF, arm drivers and
# teleop configs all use the capitalised convention (Left_, Right_, AGV_*).
# Renaming in the generated XML keeps everything compatible.
_URDF_RENAMES = [
    ('name="AICO2-4E-D1"', 'name="AICO2-Rizon4"'),
    ('"aico2_4e_d1_base_link"', '"base_link"'),
    ('"aico2_4e_d1_joint1"',    '"AGV_Joint1"'),
    ('"aico2_4e_d1_joint2"',    '"AGV_Joint2"'),
    ('"aico2_4e_d1_link1"',     '"AGV_Yaw"'),
    ('"aico2_4e_d1_link2"',     '"AGV_Pitch"'),
    ('"left_base_link"',        '"Left_link0"'),
    ('"right_base_link"',       '"Right_link0"'),
    ('"left_flange"',           '"Left_flange"'),
    ('"right_flange"',          '"Right_flange"'),
    ('"left_joint',             '"Left_joint'),
    ('"left_link',              '"Left_link'),
    ('"right_joint',            '"Right_joint'),
    ('"right_link',             '"Right_link'),
]


def _generate_urdf():
    """Generate AICO2-4E-D1 URDF from flexiv_description xacro.

    Returns (urdf_path, urdf_content).  The temp file is for
    MoveItConfigsBuilder which requires a file_path.
    """
    xacro_file = os.path.join(
        get_package_share_directory('flexiv_description'),
        'urdf', 'flexiv.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={
        'robot_type': 'AICO2-4E-D1',
        'load_gripper_left': 'true',
        'load_gripper_right': 'true',
    })
    urdf = doc.toxml()
    for old, new in _URDF_RENAMES:
        urdf = urdf.replace(old, new)
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.urdf', prefix='aico2_', delete=False)
    tmp.write(urdf)
    tmp.close()
    return tmp.name, urdf


def _include(package, filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare(package), '/launch/', filename]),
        launch_arguments=(arguments or {}).items())


def _teleop_pipeline(arm, hardware, rate_limit_scale, goto_ready,
                     *, input_topic=None, event_topic=None, device='xr',
                     hand=None, use_rviz='true'):
    """Include the moveit-teleop pipeline for one arm."""
    config_name = 'teleop.yaml' if arm == 'left' else 'right_teleop.yaml'
    config_path = os.path.join(
        get_package_share_directory('aico2_moveit_config'), 'config', config_name)
    args = {
        'robot_config': config_path,
        'joint_states_topic': '/joint_states',
        'goto_ready': goto_ready,
        'rate_limit_scale': rate_limit_scale,
        'hardware_gate_enabled': str(hardware).lower(),
        'hand': hand or ('left' if arm == 'left' else 'right'),
        'device': device,
        'use_rviz': use_rviz,
    }
    if input_topic:
        args['input_topic'] = input_topic
    if event_topic:
        args['event_topic'] = event_topic
    if device == 'external':
        # xr_bimanual_bridge publishes raw WebXR poses, like xr_device does.
        args['device_convention'] = 'webxr'
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('moveit_teleop_bringup'),
                                       '/launch/teleop.launch.py']),
        launch_arguments=args.items())


def _rt_bridge_nodes(arms_to_launch):
    """Launch rt_bridge nodes (one per arm) + joint_state_merger.

    Single-arm uses the default shm name (/aico2_rt_control) so rt_server
    needs no --shm flag.  Bimanual uses /aico2_rt_left and /aico2_rt_right.
    """
    bimanual = len(arms_to_launch) > 1
    nodes = []
    for arm in arms_to_launch:
        shm = _RT_SHM_BIMANUAL[arm] if bimanual else _RT_SHM_DEFAULT
        nodes.append(Node(
            package='aico2_rt_control',
            executable='rt_bridge',
            name='rt_bridge',
            namespace=f'{arm}_arm',
            output='screen',
            on_exit=Shutdown(),
            remappings=[('joint_states', 'joint_states')],
            parameters=[{
                'shm_name':           shm,
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

    urdf_path, urdf_content = _generate_urdf()

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

    # 2. Robot state publisher (flexiv_description xacro).
    actions.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(urdf_content, value_type=str),
            'publish_frequency': 50.0,
        }],
    ))

    # 3. MoveIt move_group.
    actions.append(_include('aico2_moveit_config', 'move_group.launch.py'))

    # 4. Trajectory adapters.
    moveit_config = (
        MoveItConfigsBuilder('AICO2', package_name=_MOVEIT_PKG)
        .robot_description(file_path=urdf_path)
        .to_moveit_configs()
    )

    arms_for_teleop = ['left', 'right'] if bimanual else [arm]

    for a in arms_for_teleop:
        joints = LEFT_JOINTS if a == 'left' else RIGHT_JOINTS
        adapter_params = {'joint_names': joints}
        if bimanual:
            adapter_params['input_topic'] = f'/xr_bimanual/{a}/input'
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
    rviz_per_arm = 'false' if bimanual else 'true'

    teleop_actions = []
    if bimanual:
        actions.append(Node(
            package='aico2_teleop',
            executable='xr_bimanual_bridge.py',
            name='xr_bimanual_bridge',
            namespace='xr_bimanual',
            output='screen',
            parameters=[{
                'port': 8443,
                'web_dir': os.path.join(
                    get_package_share_directory('aico2_teleop'), 'web'),
                'certfile': os.path.expanduser(
                    '~/.config/moveit_teleop/xr_cert.pem'),
                'keyfile': os.path.expanduser(
                    '~/.config/moveit_teleop/xr_key.pem'),
            }],
        ))
        # Each pipeline gets its own namespace (its nodes and topics would
        # otherwise collide) and its own launch-config scope (otherwise the
        # right include inherits the left's resolved move_group etc.).
        # The solver publishes to the relative <controller>/joint_trajectory;
        # the remap sends it back to the arm's absolute topic.
        for a in ('left', 'right'):
            teleop_actions.append(GroupAction([
                PushRosNamespace(f'{a}_teleop'),
                SetRemap(f'{a}_arm/joint_trajectory',
                         f'/{a}_arm/joint_trajectory'),
                _teleop_pipeline(
                    a, hardware, rate_limit_scale, home,
                    device='external',
                    input_topic=f'/xr_bimanual/{a}/input',
                    event_topic=f'/xr_bimanual/{a}/event',
                    hand=a,
                    use_rviz=rviz_per_arm),
            ]))
    else:
        teleop_actions.append(
            _teleop_pipeline(arm, hardware, rate_limit_scale, home,
                             use_rviz=rviz_per_arm))

    actions.append(TimerAction(period=teleop_delay, actions=teleop_actions))

    # 5b. Single shared RViz in bimanual mode showing the full robot.
    if bimanual:
        actions.append(TimerAction(period=teleop_delay, actions=[
            _include('aico2_moveit_config', 'moveit_rviz.launch.py'),
        ]))

    # 6. Joystick relay: right stick -> waist, left stick -> AMR
    joy_topic = ('/xr_bimanual/thumbsticks' if bimanual
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
