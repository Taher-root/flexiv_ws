#!/usr/bin/env python3
"""Bottom D456 RGBD visual odometry on its own.

Pairs with realsense.launch.py for a camera-only bringup; the full pipeline
starts the equivalent node from sensors.launch.py instead. Publishes
/camera/odom, which is the visual odometry source ekf.yaml fuses (odom1).

Two things were wrong here before and are fixed:
  - package was 'rtabmap_ros', which on Jazzy is a meta-package with no
    executables at all (`ros2 pkg executables rtabmap_ros` returns nothing);
    rgbd_odometry lives in rtabmap_odom.
  - input topics were single-segment /camera/color/... while the camera
    actually publishes under /camera/camera/..., so it received no images.

Usage:
  ros2 launch flexiv_amr_sensors visual_odometry.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node

CAMERA_BASE = "/camera/camera"


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rtabmap_odom",
            executable="rgbd_odometry",
            name="rgbd_odometry",
            output="screen",
            parameters=[{
                "frame_id": "base_link",
                "odom_frame_id": "odom_visual",
                # EKF owns odom -> base_link; this only publishes the topic.
                "publish_tf": False,
                "wait_for_transform": 0.2,
                "Odom/Strategy": "0",
                "Odom/ResetCountdown": "1",
                "OdomF2M/MaxSize": "1000",
                "Vis/CorNNType": "1",
                "Vis/MaxFeatures": "500",
                "Vis/MinInliers": "15",
            }],
            remappings=[
                ("rgb/image", f"{CAMERA_BASE}/color/image_raw"),
                ("rgb/camera_info", f"{CAMERA_BASE}/color/camera_info"),
                ("depth/image", f"{CAMERA_BASE}/depth/image_rect_raw"),
                ("odom", "/camera/odom"),
            ],
        ),
    ])
