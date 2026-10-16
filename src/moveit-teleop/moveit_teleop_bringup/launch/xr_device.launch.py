"""Meta Quest teleoperation device over WebXR.

One-time setup:

    ros2 run moveit_teleop_devices make_cert.sh 192.168.1.50
    # or: bash $(ros2 pkg prefix moveit_teleop_devices)/share/moveit_teleop_devices/scripts/make_cert.sh <ip>

Then:

    ros2 launch moveit_teleop_bringup xr_device.launch.py hand:=right

and on the Quest, browse to https://<ip>:8443, accept the certificate warning
once, and press Enter VR.

The certificate must carry the IP in subjectAltName, which make_cert.sh does;
browsers ignore the common name entirely and reject a cert without it.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_CERT_DIR = Path.home() / '.config' / 'moveit_teleop'


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'hand', default_value='right', choices=['left', 'right'],
            description='Which controller drives the arm. Both are always sent '
                        'over the wire, so switching needs no headset change.'),
        DeclareLaunchArgument('host', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='8443'),
        DeclareLaunchArgument(
            'certfile', default_value=str(DEFAULT_CERT_DIR / 'xr_cert.pem')),
        DeclareLaunchArgument(
            'keyfile', default_value=str(DEFAULT_CERT_DIR / 'xr_key.pem')),
        DeclareLaunchArgument(
            'rate_hz', default_value='90.0',
            description='Publish rate. The Quest 3S renders at 72-120 Hz; frames '
                        'are only republished when the headset clock advances, '
                        'so this is an upper bound, not a resampler.'),
        # Declared here, not only in teleop.launch.py: this file is meant to
        # be runnable on its own -- the docstring above says so -- and an
        # inherited configuration exists only when something else declared it.
        DeclareLaunchArgument(
            'feedback_topic', default_value='/teleop/feedback',
            description='Robot -> operator contact feedback: the XR bridge '
                        'turns it into a controller buzz and a tone.'),
        DeclareLaunchArgument('gripper_deadzone', default_value='0.03'),
        DeclareLaunchArgument('clutch_threshold', default_value='0.5'),
        DeclareLaunchArgument('namespace', default_value=''),

        Node(
            package='moveit_teleop_devices',
            executable='xr_device',
            name='xr_device',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'hand': LaunchConfiguration('hand'),
                'feedback_topic': LaunchConfiguration('feedback_topic'),
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'certfile': LaunchConfiguration('certfile'),
                'keyfile': LaunchConfiguration('keyfile'),
                'rate_hz': LaunchConfiguration('rate_hz'),
                'gripper_deadzone': LaunchConfiguration('gripper_deadzone'),
                'clutch_threshold': LaunchConfiguration('clutch_threshold'),
            }],
        ),
    ])
