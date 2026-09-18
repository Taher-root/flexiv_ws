"""
joy_teleop.launch.py - Drive the AICO2 base with an 8BitDo controller.

Launches:
  1. joy_node          - reads /dev/input/jsX, publishes sensor_msgs/Joy on /joy
  2. teleop_twist_joy  - converts /joy -> geometry_msgs/Twist on /cmd_vel

/cmd_vel is exactly what `robokit_velocity_controller` (flexiv_amr_driver)
already subscribes to, so this launch file does NOT start the base driver
itself - run it alongside amr_driver_robokit.launch.py (or full_system /
mapping_robokit) which already brings that up.

Usage:
  # 1) Plug in / pair the 8BitDo pad, switch it to X-input mode
  #    (hold START + X, or flip the mode switch to "X").
  # 2) Make sure the base driver is running, e.g.:
  ros2 launch flexiv_amr_driver amr_driver_robokit.launch.py
  # 3) In another terminal:
  ros2 launch flexiv_amr_driver joy_teleop.launch.py

  # Optional: point at a different joystick device or config
  ros2 launch flexiv_amr_driver joy_teleop.launch.py device:=/dev/input/js1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('flexiv_amr_driver')
    default_config = os.path.join(pkg_share, 'config', 'joy_teleop_8bitdo.yaml')

    config_file = LaunchConfiguration('config_file')
    joy_device = LaunchConfiguration('device')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='Path to joy/teleop_twist_joy params yaml'),
        DeclareLaunchArgument(
            'device', default_value='/dev/input/js0',
            description='Joystick device path for the 8BitDo controller'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/cmd_vel',
            description='Twist output topic (must match base driver subscription)'),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[config_file, {'device_name': joy_device}],
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[config_file],
            remappings=[('/cmd_vel', cmd_vel_topic)],
        ),
    ])
