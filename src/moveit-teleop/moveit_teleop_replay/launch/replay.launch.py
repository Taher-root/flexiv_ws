"""Review recorded episodes in a dedicated RViz.

    ros2 launch moveit_teleop_replay replay.launch.py \
        replay_urdf_xacro:=<path>.urdf.xacro \
        replay_srdf_xacro:=<path>.srdf.xacro \
        replay_xacro_args:="<the args the robot bringup uses>"

Use this when nothing else is running. If a teleop session is already up, do NOT
run this alongside it -- teleop.launch.py has replay:=true by default and puts
the same panel and the same preview robot in the RViz it already opened, which
is one window instead of two fighting for the screen.

The three model arguments are REQUIRED and have no default: the preview robot
must be the arm the episode was recorded on, nothing in the bag records which
arm that was, and a mismatch shows a plausible-looking arm in the wrong pose
with no error anywhere. Their values are exactly the robot config's
files.preview_urdf / preview_srdf / preview_xacro_args -- which is what the
teleop launch passes, so prefer

    ros2 launch moveit_teleop_bringup teleop.launch.py \
        robot_config_package:=<arm>_moveit_config replay:=true

and let the robot config supply them.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from replay_nodes_support import descriptions  # noqa: E402


def _spawn(context):
    urdf, _ = descriptions(context)
    return [
        # Only here for TF: it gives the scene a 'world' frame and lets the
        # optional solid RobotModel display resolve its links. The preview
        # robot needs none of this -- it is drawn from the DisplayRobotState
        # message -- but with nothing else running there is no TF at all.
        #
        # Its robot_description output goes nowhere on purpose: replay_nodes
        # already latches that topic, and two publishers of one latched topic
        # is a race over which description wins.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='replay_robot_state_publisher',
            output='screen',
            parameters=[{'robot_description':
                         ParameterValue(urdf, value_type=str)}],
            remappings=[
                ('robot_description', '/replay_rsp_description_unused'),
                ('joint_states', LaunchConfiguration('replay_joint_states_topic')),
                # robot_state_publisher in Jazzy has no frame_prefix parameter,
                # and two publishers of the SAME frame names interleave rather
                # than merge, so the tree flickers between two robots.
                ('/tf', '/replay/tf'),
                ('/tf_static', '/replay/tf_static'),
            ],
        ),
        Node(
            package='moveit_teleop_replay',
            executable='tf_prefixer',
            name='tf_prefixer',
            output='screen',
            parameters=[{
                'prefix': LaunchConfiguration('tf_prefix'),
                'root_frame': LaunchConfiguration('root_frame'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='replay_rviz',
            output='log',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory('moveit_teleop_replay')
    forwarded = [
        'replay_episodes_dir', 'replay_urdf_xacro', 'replay_srdf_xacro',
        'replay_xacro_args', 'replay_joint_states_topic',
        'replay_target_topic', 'replay_rate_hz',
        'replay_autoload_latest', 'replay_preview_color',
    ]
    return LaunchDescription([
        DeclareLaunchArgument('replay_episodes_dir',
                              default_value='~/teleop_episodes'),

        # The PREVIEW robot's model: REQUIRED, with no default, and declared
        # here only so this launch can forward it. See the module docstring.
        DeclareLaunchArgument('replay_urdf_xacro', default_value=''),
        DeclareLaunchArgument('replay_srdf_xacro', default_value=''),
        DeclareLaunchArgument('replay_xacro_args', default_value=''),
        DeclareLaunchArgument('replay_joint_states_topic',
                              default_value='/replay/joint_states'),
        DeclareLaunchArgument('replay_target_topic', default_value='/replay/target_pose'),
        DeclareLaunchArgument('replay_rate_hz', default_value='0.0'),
        DeclareLaunchArgument('replay_autoload_latest', default_value='true'),
        DeclareLaunchArgument('replay_preview_color',
                              default_value='[0.15, 0.8, 0.35, 1.0]'),
        DeclareLaunchArgument('tf_prefix', default_value='replay/'),
        DeclareLaunchArgument('root_frame', default_value='world'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(share, 'config', 'replay.rviz')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'replay_nodes.launch.py')),
            launch_arguments={k: LaunchConfiguration(k)
                              for k in forwarded}.items()),

        OpaqueFunction(function=_spawn),
    ])
