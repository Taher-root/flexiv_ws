"""Standalone visual odometry, paired with this package's realsense.launch.py.

Confirmed on the actual Jazzy install (2026-09-19): `ros2 pkg executables
rtabmap_ros` returns nothing — rtabmap_ros is a meta-package on Jazzy with no
executables of its own; rgbd_odometry now lives in rtabmap_odom. Fixed below.

Note this file's topics (/camera/color/..., /camera/odom) assume the
single-segment camera namespace that realsense.launch.py's rs_launch.py
include also uses. That's a different convention from sensors.launch.py
(the one used by hardware_test/mapping/navigation/full_system), which uses
realsense2_camera's newer default double-segment namespace
(/camera/camera/color/...). If you run this file against the camera driver
sensors.launch.py starts instead of realsense.launch.py's, it will not
receive any images.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom_visual',
                'publish_tf': False,
                'wait_for_transform': 0.2,
                'Odom/Strategy': '0',
                'Odom/ResetCountdown': '1',
                'OdomF2M/MaxSize': '1000',
                'Vis/CorNNType': '1',
                'Vis/MaxFeatures': '500',
                'Vis/MinInliers': '15',
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_rect_raw'),
                ('odom', '/camera/odom')
            ]
        ),
    ])