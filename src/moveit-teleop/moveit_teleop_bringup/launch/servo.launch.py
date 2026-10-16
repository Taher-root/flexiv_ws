"""Launch moveit_servo against an existing MoveIt stack.

Run this ALONGSIDE a robot's own MoveIt bringup, not instead of it. For the
Piper that is the vendored:

    ros2 launch agx_arm_moveit demo.launch.py auto_control_gate:=false

This file is deliberately robot-agnostic: the robot enters only through
``moveit_config_package``/``robot_name`` and the xacro mappings, so pointing it
at a different arm is a launch argument rather than a code change.

Servo will not act on pose commands until its command type is switched:

    ros2 service call /servo_node/switch_command_type \
        moveit_msgs/srv/ServoCommandType "{command_type: 2}"

The mode manager does this automatically; it is spelled out here because a
silent no-op is the first thing to check when servo appears dead.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _parse_mappings(text: str) -> dict:
    """Parse ``"a:=1 b:=2 3 4"`` into ``{"a": "1", "b": "2 3 4"}``.

    Values may contain spaces: xacro arguments are routinely vectors written
    as ``tcp_offset_xyz:=0 0 0.1425``. A token without ``:=`` therefore
    continues the previous value rather than being an error -- splitting purely
    on whitespace rejects every offset the robot actually needs.

    Mappings are passed as one string so this launch file needs no knowledge of
    which xacro arguments a particular robot happens to define.
    """
    mappings, key = {}, None
    for token in text.split():
        if ':=' in token:
            key, _, value = token.partition(':=')
            mappings[key] = value
        elif key is None:
            raise ValueError(
                f'malformed xacro mappings {text!r}: {token!r} appears before '
                'any name:=value')
        else:
            mappings[key] = f'{mappings[key]} {token}'.strip()
    return mappings


def _launch_setup(context):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    config_package = LaunchConfiguration('moveit_config_package').perform(context)
    urdf_file = LaunchConfiguration('urdf_file').perform(context)
    srdf_file = LaunchConfiguration('srdf_file').perform(context)
    kinematics_file = LaunchConfiguration('kinematics_file').perform(context)
    joint_limits_file = LaunchConfiguration('joint_limits_file').perform(context)
    mappings = _parse_mappings(
        LaunchConfiguration('xacro_mappings').perform(context))
    move_group_name = LaunchConfiguration('move_group_name').perform(context)
    servo_config_package = LaunchConfiguration(
        'servo_config_package').perform(context)
    servo_config_file = LaunchConfiguration('servo_config_file').perform(context)

    # Rebuild the same MoveIt config the robot's own bringup uses. Servo needs
    # the description, semantics, kinematics and joint limits as its own
    # parameters -- it cannot borrow them from move_group over the wire.
    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name=config_package)
        .robot_description(file_path=urdf_file, mappings=mappings)
        .robot_description_semantic(file_path=srdf_file, mappings=mappings)
        .robot_description_kinematics(file_path=kinematics_file)
        .joint_limits(file_path=joint_limits_file)
        .to_moveit_configs()
    )

    # Everything from servo.yaml must sit under a "moveit_servo" prefix, which
    # is where generate_parameter_library's ParamListener looks for it.
    servo_params = {
        'moveit_servo': ParameterBuilder(servo_config_package)
        .yaml(servo_config_file)
        .to_dict()
    }

    # These two are TOP-LEVEL node parameters, deliberately not inside the
    # moveit_servo block: AccelerationLimitedPlugin reads them directly, and
    # nesting them makes servo fail to load the smoother.
    smoothing_params = {
        'update_period': 0.01,
        'planning_group_name': move_group_name,
    }

    return [
        Node(
            package='moveit_servo',
            executable='servo_node',
            name='servo_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                servo_params,
                smoothing_params,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name', default_value='agx_arm',
            description='Robot name passed to MoveItConfigsBuilder.'),
        DeclareLaunchArgument(
            'moveit_config_package', default_value='agx_arm_moveit',
            description='Package holding the MoveIt config for this robot.'),
        DeclareLaunchArgument(
            'urdf_file', default_value='config/agx_arm.urdf.xacro'),
        DeclareLaunchArgument(
            'srdf_file', default_value='config/agx_arm.srdf.xacro'),
        DeclareLaunchArgument(
            'kinematics_file', default_value='config/kinematics.yaml'),
        DeclareLaunchArgument(
            'joint_limits_file', default_value='config/joint_limits.yaml'),
        DeclareLaunchArgument(
            'xacro_mappings',
            default_value='arm_type:=piper effector_type:=none revo2_type:=left',
            description='Space-separated name:=value xacro args. MUST match the '
                        'robot bringup exactly, or servo plans against a '
                        'different robot than the one being commanded.'),
        DeclareLaunchArgument(
            'move_group_name', default_value='arm',
            description='Planning group servo actuates.'),
        DeclareLaunchArgument(
            'servo_config_package', default_value='moveit_teleop_bringup'),
        DeclareLaunchArgument(
            'servo_config_file', default_value='config/servo.yaml'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Must match the robot bringup namespace.'),
        OpaqueFunction(function=_launch_setup),
    ])
