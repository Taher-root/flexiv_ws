from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aico2_vr_teleop',
            executable='vr_bridge',
            name='vr_bridge',
            output='screen',
        ),
    ])
