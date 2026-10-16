"""The full-IK solver, as an alternative to moveit_servo.

Consumes the same PoseStamped the retargeter publishes and drives the same
JointTrajectoryController, so it slots into the pipeline where servo was.
Launched by teleop.launch.py when ``solver:=ik``.
"""

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('move_group', default_value='arm'),
        DeclareLaunchArgument('description_source', default_value='/move_group'),
        DeclareLaunchArgument(
            'nominal_state', default_value='ready_pose',
            description='SRDF group state the solver is regularised toward. '
                        'Must be well conditioned -- the node checks and warns.'),
        DeclareLaunchArgument('joint_states_topic',
                              default_value='/control/joint_states'),
        DeclareLaunchArgument('solve_rate', default_value='90.0'),
        DeclareLaunchArgument('weight_position', default_value='50.0'),
        DeclareLaunchArgument(
            'weight_orientation', default_value='10.0',
            description='Raised from xr_teleoperate\'s 1.0: at 50:1 the '
                        'steady-state orientation residual is ~0.8 deg.'),
        DeclareLaunchArgument(
            'weight_regularization', default_value='0.005',
            description='Sole source of steady-state tracking error. On a '
                        '6-DOF arm the pose fully determines q, so this can '
                        'only pull away from the target.'),
        DeclareLaunchArgument('weight_smoothness', default_value='0.1'),
        DeclareLaunchArgument('max_iter', default_value='30'),
        DeclareLaunchArgument('rate_limit_scale', default_value='0.3'),
        DeclareLaunchArgument('max_joint_jump', default_value='0.35'),
        # On a refused jump, retry once pulled toward the last accepted
        # solution. 0 disables it and the arm holds instead.
        DeclareLaunchArgument('jump_retry_smoothness',
                              default_value='50.0'),
        DeclareLaunchArgument('use_moveit_limits', default_value='true'),
        DeclareLaunchArgument('use_moveit_velocity_scaling', default_value='false'),
        DeclareLaunchArgument(
            'planes', default_value='[]',
            description='Virtual walls as a flat [nx, ny, nz, offset, ...] '
                        'enforcing n . p >= offset on the tool. Constraints, '
                        'so the tool slides along them rather than stopping.'),
        # 'none', not 'off': YAML reads off/on/yes/no as booleans, so 'off'
        # would reach the node as False and fail a string parameter.
        DeclareLaunchArgument(
            'scene_config', default_value='',
            description='Planning-scene YAML. Each box that declares a '
                        'wall face becomes a constraint in the solve, '
                        'derived from that box bounds so it cannot '
                        'drift from the collision object.'),
        DeclareLaunchArgument(
            'collision_policy', default_value='stop',
            choices=['none', 'stop'],
            description="'stop' runs at full speed to contact and refuses to "
                        'enter a collision. Servo-style proximity slowing is '
                        'deliberately not offered.'),

        Node(
            package='moveit_teleop_ik',
            executable='ik_node',
            name='ik_node',
            output='screen',
            parameters=[{
                'move_group': LaunchConfiguration('move_group'),
                'description_source': LaunchConfiguration('description_source'),
                'nominal_state': LaunchConfiguration('nominal_state'),
                'joint_states_topic': LaunchConfiguration('joint_states_topic'),
                'solve_rate': LaunchConfiguration('solve_rate'),
                'weight_position': LaunchConfiguration('weight_position'),
                'weight_orientation': LaunchConfiguration('weight_orientation'),
                'weight_regularization':
                    LaunchConfiguration('weight_regularization'),
                'weight_smoothness': LaunchConfiguration('weight_smoothness'),
                'max_iter': LaunchConfiguration('max_iter'),
                'rate_limit_scale': LaunchConfiguration('rate_limit_scale'),
                'max_joint_jump': LaunchConfiguration('max_joint_jump'),
                'jump_retry_smoothness':
                    LaunchConfiguration('jump_retry_smoothness'),
                'use_moveit_limits': LaunchConfiguration('use_moveit_limits'),
                'use_moveit_velocity_scaling':
                    LaunchConfiguration('use_moveit_velocity_scaling'),
                'scene_config': LaunchConfiguration('scene_config'),
                'collision_policy': LaunchConfiguration('collision_policy'),
                # Typed explicitly so an EMPTY list is still a double array;
                # launch cannot infer the item type of '[]'.
                'planes': ParameterValue(LaunchConfiguration('planes'),
                                         value_type=List[float]),
            }],
        ),
    ])
