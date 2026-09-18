"""
rtabmap_mapping.launch.py - RTAB-Map SLAM with D456 camera + LiDAR

Uses rgbd_sync to bundle camera images into a single RGBDImage message,
preventing resource contention with visual odometry nodes that also
subscribe to the raw image topics.

Pipeline:
  Camera raw topics -> rgbd_sync -> /rgbd_image (bundled) -> rtabmap
  This avoids two nodes fighting over the same SHM image streams.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_nav2 = get_package_share_directory('flexiv_amr_nav2')
    rtabmap_config = os.path.join(pkg_nav2, 'config', 'rtabmap_params.yaml')

    return LaunchDescription([
        # --------------------------------------------------------
        # rgbd_sync: bundles RGB + Depth + CameraInfo into one msg
        # This is the ONLY node that subscribes to raw camera images
        # for RTAB-Map SLAM (visual odom already has its own sub).
        # --------------------------------------------------------
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            output='screen',
            parameters=[{
                'approx_sync': True,
                'approx_sync_max_interval': 0.05,
                'queue_size': 10,
            }],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/depth/image_rect_raw'),
                ('rgbd_image', '/rgbd_image'),
            ],
        ),

        # --------------------------------------------------------
        # RTAB-Map SLAM: subscribes to bundled /rgbd_image + odom + scan
        # Does NOT touch raw image topics directly.
        # --------------------------------------------------------
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_config],
            remappings=[
                ('rgbd_image', '/rgbd_image'),
                ('odom', '/odometry/filtered'),
                ('scan', '/scan/depth'),
                ('map', '/map'),
            ],
        ),
    ])
