#!/usr/bin/env python3
"""RTAB-Map in pure localization mode against a pre-built database.

Publishes map -> odom TF and /map — the same job AMCL + map_server do in
localization.launch.py. Do not run both at once; flexiv_amr_bringup's
navigation.launch.py picks one via localization_backend.

Defaults to maps/rtabmap_backup.db — the larger, later survey (53 MB,
2026-08-31) confirmed as the good map, not the smaller maps/rtabmap.db
(1.8 MB, 2026-08-04) that happens to also be checked in.

Mem/IncrementalMemory and delete_db_on_start are forced here regardless of
what rtabmap_params.yaml says, so this launch file can never itself add
nodes to or wipe the database you're localizing against — only
rtabmap_mapping.launch.py (mapping mode) writes.

Usage:
  ros2 launch flexiv_amr_nav2 rtabmap_localization.launch.py
  ros2 launch flexiv_amr_nav2 rtabmap_localization.launch.py rtabmap_db:=/path/to/other.db
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('flexiv_amr_nav2')
    rtabmap_config = PathJoinSubstitution([pkg_share, 'config', 'rtabmap_params.yaml'])
    default_db = PathJoinSubstitution([pkg_share, 'maps', 'rtabmap_backup.db'])

    rtabmap_db = LaunchConfiguration('rtabmap_db')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rtabmap_db', default_value=default_db,
            description='Database to localize against (read-only here)',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock',
        ),

        # Same bundling node rtabmap_mapping.launch.py uses — RTAB-Map only
        # ever subscribes to the bundled /rgbd_image, not raw camera topics.
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            output='screen',
            parameters=[{
                'approx_sync': True,
                'approx_sync_max_interval': 0.05,
                'queue_size': 10,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/depth/image_rect_raw'),
                ('rgbd_image', '/rgbd_image'),
            ],
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[
                rtabmap_config,
                {
                    'database_path': rtabmap_db,
                    'Mem/IncrementalMemory': 'false',
                    'delete_db_on_start': False,
                    'use_sim_time': use_sim_time,
                },
            ],
            remappings=[
                ('rgbd_image', '/rgbd_image'),
                ('odom', '/odometry/filtered'),
                ('scan', '/scan/depth'),
                ('map', '/map'),
            ],
        ),
    ])
