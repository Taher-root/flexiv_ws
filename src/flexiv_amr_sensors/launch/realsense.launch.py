#!/usr/bin/env python3
"""Bottom D456 camera on its own, plus depth-to-laserscan.

For bringing up just the camera — checking it enumerates, framing it for a
mapping run, debugging image topics — without the chassis sensors, scan
merger or visual odometry that sensors.launch.py also starts. The full
pipeline (hardware_test / mapping / navigation / full_system) uses
sensors.launch.py instead; this is the standalone tool.

Topics match the rest of the stack: realsense2_camera's default namespace
puts the bottom camera at /camera/camera/..., which is what ekf.yaml,
rtabmap_mapping.launch.py and rtabmap_localization.launch.py all expect.
(This file previously used single-segment /camera/... paths and published
its scan to /scan rather than /scan/depth, so nothing downstream saw it.)

Usage:
  ros2 launch flexiv_amr_sensors realsense.launch.py
  ros2 launch flexiv_amr_sensors realsense.launch.py pointcloud:=true
  ros2 launch flexiv_amr_sensors realsense.launch.py scan_topic:=/scan
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Bottom D456. Same serial sensors.launch.py pins, so the two files cannot
# silently grab different cameras on a two-camera robot.
CAMERA_SERIAL = "_333422304124"
CAMERA_BASE = "/camera/camera"


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("serial_no", default_value=CAMERA_SERIAL,
                              description="Bottom D456 serial number"),
        DeclareLaunchArgument("enable_depth", default_value="true"),
        DeclareLaunchArgument("enable_color", default_value="true"),
        DeclareLaunchArgument("enable_infra", default_value="false"),
        DeclareLaunchArgument("enable_gyro", default_value="true"),
        DeclareLaunchArgument("enable_accel", default_value="true"),
        # '1' is the numeric form of linear_interpolation, and is what
        # sensors.launch.py passes — kept identical so /camera/imu behaves the
        # same however the camera was started.
        DeclareLaunchArgument("unite_imu_method", default_value="1"),
        DeclareLaunchArgument("pointcloud", default_value="false",
                              description="Publish the organised point cloud"),
        DeclareLaunchArgument("scan_topic", default_value="/scan/depth",
                              description="Where depth-to-laserscan publishes"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"
                ])
            ),
            launch_arguments={
                "serial_no": LaunchConfiguration("serial_no"),
                "initial_reset": "false",
                "enable_depth": LaunchConfiguration("enable_depth"),
                "enable_color": LaunchConfiguration("enable_color"),
                "enable_infra": LaunchConfiguration("enable_infra"),
                "enable_gyro": LaunchConfiguration("enable_gyro"),
                "enable_accel": LaunchConfiguration("enable_accel"),
                "unite_imu_method": LaunchConfiguration("unite_imu_method"),
                "depth_module.profile": "640x480x30",
                "rgb_camera.profile": "640x480x30",
                "align_depth.enable": "true",
                "pointcloud.enable": LaunchConfiguration("pointcloud"),
            }.items(),
        ),

        Node(
            package="depthimage_to_laserscan",
            executable="depthimage_to_laserscan_node",
            name="depthimage_to_laserscan",
            remappings=[
                ("depth", f"{CAMERA_BASE}/depth/image_rect_raw"),
                ("depth_camera_info", f"{CAMERA_BASE}/depth/camera_info"),
                ("scan", LaunchConfiguration("scan_topic")),
            ],
            parameters=[{
                "output_frame": "camera_depth_frame",
                "scan_height": 10,
                "range_min": 0.3,
                "range_max": 10.0,
            }],
        ),
    ])
