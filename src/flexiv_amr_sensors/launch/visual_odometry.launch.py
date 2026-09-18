from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='rtabmap_ros',
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