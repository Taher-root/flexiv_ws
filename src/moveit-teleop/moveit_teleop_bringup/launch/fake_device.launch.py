"""Run the scripted fake teleoperation device on its own.

Deliberately standalone: no robot, no MoveIt, no controllers. Being able to
run and inspect the input source in isolation is what makes the later
integration steps debuggable, because it removes any question about whether a
problem originates upstream of the retargeter.

    ros2 launch moveit_teleop_bringup fake_device.launch.py pattern:=clutch_storm
"""

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Kept in sync with moveit_teleop_devices.fake_device.PATTERNS. Declared here
# as choices so a typo fails at launch with a readable message rather than
# inside the node.
PATTERNS = ['circle', 'square', 'ramp', 'wobble', 'clutch_storm', 'dropout', 'static']


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pattern', default_value='circle', choices=PATTERNS,
            description='Scripted motion pattern to publish.'),
        DeclareLaunchArgument(
            'rate_hz', default_value='100.0',
            description='Publish rate of the input stream.'),
        DeclareLaunchArgument(
            'period', default_value='6.0',
            description='Seconds per cycle of the pattern.'),
        DeclareLaunchArgument(
            'offset', default_value='[0.0, 0.0, 0.0]',
            description='Device-frame offset the pattern is centred on, '
                        'ramped in. Use it to put a small circle in '
                        'front of the robot and below a table.'),
        DeclareLaunchArgument('offset_ramp', default_value='3.0'),
        DeclareLaunchArgument(
            'start_delay', default_value='0.0',
            description='Hold still this long before moving. Must outlast '
                        'homing, or the motion is absorbed into the '
                        'retargeting anchor and has no effect.'),
        DeclareLaunchArgument(
            'plane', default_value='xy',
            description="Plane the circle/square is swept in. 'xy' is "
                        "horizontal and can never reach a table; 'xz' "
                        'sweeps vertically and drives through one.'),
        DeclareLaunchArgument(
            'radius', default_value='0.1',
            description='Motion amplitude in metres.'),
        DeclareLaunchArgument(
            'emit_episode_events', default_value='false',
            description='Also drive a record/save cycle, for testing the recorder.'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Optional namespace, to match a namespaced robot bringup.'),

        Node(
            package='moveit_teleop_devices',
            executable='fake_device',
            name='fake_device',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'pattern': LaunchConfiguration('pattern'),
                'rate_hz': LaunchConfiguration('rate_hz'),
                'period': LaunchConfiguration('period'),
                'radius': LaunchConfiguration('radius'),
                'plane': LaunchConfiguration('plane'),
                'offset': ParameterValue(
                    LaunchConfiguration('offset'), value_type=List[float]),
                'offset_ramp': LaunchConfiguration('offset_ramp'),
                'start_delay': LaunchConfiguration('start_delay'),
                'emit_episode_events': LaunchConfiguration('emit_episode_events'),
            }],
        ),
    ])
