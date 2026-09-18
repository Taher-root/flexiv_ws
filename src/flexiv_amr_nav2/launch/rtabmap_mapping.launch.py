#!/usr/bin/env python3
"""RTAB-Map SLAM — builds a NEW map (D456 camera + LiDAR).

Uses rgbd_sync to bundle camera images into a single RGBDImage message,
preventing resource contention with visual odometry nodes that also
subscribe to the raw image topics.

Pipeline:
  Camera raw topics -> rgbd_sync -> /rgbd_image (bundled) -> rtabmap
  This avoids two nodes fighting over the same SHM image streams.

This is the mapping counterpart to rtabmap_localization.launch.py. The mode
keys (Mem/IncrementalMemory, database_path, delete_db_on_start) are forced
here rather than read from rtabmap_params.yaml, which holds only shared
tuning — see the note at the top of that file.

Writes to rtabmap_db, which defaults to ~/.ros/aico2_map.db and is
deliberately NOT the checked-in localization database: a mapping run must
never grow or corrupt the map that navigation localizes against. Pass
delete_db_on_start:=true to start a genuinely fresh map; the default appends
to whatever is already at that path so an accidental relaunch cannot wipe a
map you just spent time building.

Usage:
  ros2 launch flexiv_amr_nav2 rtabmap_mapping.launch.py
  ros2 launch flexiv_amr_nav2 rtabmap_mapping.launch.py delete_db_on_start:=true
  ros2 launch flexiv_amr_nav2 rtabmap_mapping.launch.py rtabmap_db:=/tmp/try2.db

When the map is good, point navigation at it:
  ros2 launch flexiv_amr_bringup navigation.launch.py rtabmap_db:=~/.ros/aico2_map.db
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rtabmap_config = PathJoinSubstitution(
        [FindPackageShare("flexiv_amr_nav2"), "config", "rtabmap_params.yaml"]
    )
    default_db = os.path.join(os.path.expanduser("~"), ".ros", "aico2_map.db")

    rtabmap_db = LaunchConfiguration("rtabmap_db")
    delete_db_on_start = LaunchConfiguration("delete_db_on_start")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "rtabmap_db", default_value=default_db,
            description="Database this mapping run writes to (never the "
                        "checked-in localization map)",
        ),
        DeclareLaunchArgument(
            "delete_db_on_start", default_value="false",
            description="true starts a fresh map, discarding rtabmap_db's "
                        "current contents; false appends to it",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use simulation clock",
        ),

        # --------------------------------------------------------
        # rgbd_sync: bundles RGB + Depth + CameraInfo into one msg
        # This is the ONLY node that subscribes to raw camera images
        # for RTAB-Map SLAM (visual odom already has its own sub).
        # --------------------------------------------------------
        Node(
            package="rtabmap_sync",
            executable="rgbd_sync",
            name="rgbd_sync",
            output="screen",
            parameters=[{
                "approx_sync": True,
                "approx_sync_max_interval": 0.05,
                "queue_size": 10,
                "use_sim_time": use_sim_time,
            }],
            remappings=[
                ("rgb/image", "/camera/camera/color/image_raw"),
                ("rgb/camera_info", "/camera/camera/color/camera_info"),
                ("depth/image", "/camera/camera/depth/image_rect_raw"),
                ("rgbd_image", "/rgbd_image"),
            ],
        ),

        # --------------------------------------------------------
        # RTAB-Map SLAM: subscribes to bundled /rgbd_image + odom + scan
        # Does NOT touch raw image topics directly.
        # --------------------------------------------------------
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[
                rtabmap_config,
                {
                    # Mapping mode: grow the map as new places are seen.
                    "Mem/IncrementalMemory": "true",
                    "database_path": rtabmap_db,
                    "delete_db_on_start": delete_db_on_start,
                    "use_sim_time": use_sim_time,
                },
            ],
            remappings=[
                ("rgbd_image", "/rgbd_image"),
                ("odom", "/odometry/filtered"),
                ("scan", "/scan/depth"),
                ("map", "/map"),
            ],
        ),
    ])
