"""The replay machinery, without any RViz of its own.

Included BOTH by replay.launch.py (which adds its own RViz for reviewing
episodes with nothing else running) and by the teleop launch (which already has
an RViz, and just wants the panel and the preview robot in it).

Publishing the descriptions here rather than leaning on whatever move_group
happens to expose keeps the two cases identical: the preview robot is built
from the same xacro arguments in both, so an episode looks the same whether you
review it beside a live arm or on its own.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from replay_nodes_support import SRDF_TOPIC, URDF_TOPIC, descriptions


def _spawn(context):
    urdf, srdf = descriptions(context)
    return [
        # MoveIt's RobotState display builds its own RobotModel, so it needs
        # the SRDF as well as the URDF. Nothing publishes a semantic
        # description outside a running move_group.
        Node(
            package='moveit_teleop_replay', executable='description_publisher',
            name='replay_urdf_publisher', output='screen',
            parameters=[{'topic': URDF_TOPIC,
                         'content': ParameterValue(urdf, value_type=str)}],
        ),
        Node(
            package='moveit_teleop_replay', executable='description_publisher',
            name='replay_srdf_publisher', output='screen',
            parameters=[{'topic': SRDF_TOPIC,
                         'content': ParameterValue(srdf, value_type=str)}],
        ),
        Node(
            package='moveit_teleop_replay', executable='episode_player',
            name='episode_player', output='screen',
            parameters=[{
                'episodes_dir': LaunchConfiguration('replay_episodes_dir'),
                'joint_states_topic': LaunchConfiguration('replay_joint_states_topic'),
                'target_topic': LaunchConfiguration('replay_target_topic'),
                'robot_description_topic': URDF_TOPIC,
                'rate_hz': LaunchConfiguration('replay_rate_hz'),
                'autoload_latest': LaunchConfiguration('replay_autoload_latest'),
                # Typed explicitly: launch cannot infer the element type of
                # a list given as a string, and an untyped one arrives as
                # a STRING where a DOUBLE_ARRAY is declared.
                'preview_color': ParameterValue(
                    LaunchConfiguration('replay_preview_color'),
                    value_type=List[float]),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        # No default: the preview robot MUST be the recorded arm, and no arm
        # is a sensible guess. The teleop launch fills these from the robot
        # config's files.preview_* keys.
        DeclareLaunchArgument(
            'replay_urdf_xacro', default_value='',
            description='REQUIRED. URDF xacro (or plain URDF) for the PREVIEW '
                        'robot, which must be the arm the episode was '
                        'recorded on.'),
        DeclareLaunchArgument(
            'replay_srdf_xacro', default_value='',
            description='REQUIRED. SRDF xacro for the preview robot. MoveIt\'s '
                        'RobotState display builds its own RobotModel and '
                        'needs both.'),
        DeclareLaunchArgument(
            'replay_xacro_args', default_value='',
            description='Arguments passed to both xacros, verbatim. Values '
                        'containing spaces must be quoted here.'),
        DeclareLaunchArgument('replay_episodes_dir', default_value='~/teleop_episodes'),
        DeclareLaunchArgument('replay_joint_states_topic',
                              default_value='/replay/joint_states'),
        DeclareLaunchArgument('replay_target_topic', default_value='/replay/target_pose'),
        DeclareLaunchArgument('replay_rate_hz', default_value='0.0'),
        DeclareLaunchArgument('replay_autoload_latest', default_value='true'),
        DeclareLaunchArgument(
            'replay_preview_color', default_value='[0.15, 0.8, 0.35, 1.0]',
            description='RGBA the preview robot is drawn in.'),
        OpaqueFunction(function=_spawn),
    ])
