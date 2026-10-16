"""Fake device + retargeter, open loop.

Step 3 of bring-up: this publishes a pose target and broadcasts it as TF, but
is NOT connected to servo. Run it on top of the robot's MoveIt bringup and
watch the 'teleop_target' frame next to the real end effector in RViz:

    ros2 launch agx_arm_moveit demo.launch.py auto_control_gate:=false
    ros2 launch moveit_teleop_bringup retarget.launch.py pattern:=circle

Verify by eye before connecting anything: the target should trace the pattern,
freeze when declutched, and change size with translation_scale. Debugging
coordinate conventions here costs minutes; debugging them through a moving
robot costs hours.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pattern', default_value='circle'),
        DeclareLaunchArgument('period', default_value='6.0'),
        DeclareLaunchArgument('radius', default_value='0.1'),
        DeclareLaunchArgument('move_group', default_value='arm'),
        DeclareLaunchArgument(
            'ee_link', default_value='',
            description='Empty means derive from the SRDF end effector.'),
        DeclareLaunchArgument(
            'planning_frame', default_value='',
            description='Empty means use the URDF root link.'),
        DeclareLaunchArgument('translation_scale', default_value='0.5'),
        DeclareLaunchArgument('rotation_scale', default_value='1.0'),
        DeclareLaunchArgument('translation_ref', default_value='base'),
        DeclareLaunchArgument('rotation_ref', default_value='tool'),
        DeclareLaunchArgument(
            'device_convention', default_value='ros',
            description="Device axes -> ROS axes: 'ros' or 'webxr'."),
        DeclareLaunchArgument('smoothing_alpha', default_value='1.0'),
        DeclareLaunchArgument('max_translation_step', default_value='0.05'),
        DeclareLaunchArgument('description_source', default_value='/move_group'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Show the commanded target next to the real EE. This is '
                        'the entire point of running the retargeter open loop, '
                        'so it defaults on.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('moveit_teleop_bringup'),
                'launch', 'fake_device.launch.py'])),
            launch_arguments={
                'pattern': LaunchConfiguration('pattern'),
                'period': LaunchConfiguration('period'),
                'radius': LaunchConfiguration('radius'),
            }.items()),

        Node(
            package='moveit_teleop_core',
            executable='retarget_node',
            name='retarget_node',
            output='screen',
            parameters=[{
                'move_group': LaunchConfiguration('move_group'),
                'ee_link': LaunchConfiguration('ee_link'),
                'planning_frame': LaunchConfiguration('planning_frame'),
                'description_source': LaunchConfiguration('description_source'),
                'input_topic': '/fake_device/input',
                'translation_scale': LaunchConfiguration('translation_scale'),
                'rotation_scale': LaunchConfiguration('rotation_scale'),
                'translation_ref': LaunchConfiguration('translation_ref'),
                'rotation_ref': LaunchConfiguration('rotation_ref'),
                'device_convention': LaunchConfiguration('device_convention'),
                'smoothing_alpha': LaunchConfiguration('smoothing_alpha'),
                'max_translation_step': LaunchConfiguration('max_translation_step'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='teleop_rviz',
            output='log',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('moveit_teleop_bringup'),
                'config', 'teleop.rviz'])],
        ),
    ])
