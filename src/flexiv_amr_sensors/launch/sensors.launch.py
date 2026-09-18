#!/usr/bin/env python3
"""Chassis sensors + RealSense camera(s) + depth-to-laserscan + scan merger.

One camera by default (the bottom D456). full_system brings up the head-mounted
top camera as well with use_top_camera:=true, which adds /scan/top to the merged
scan and a second visual odometry node.

Publishes:
  /scan/nav, /scan/avoid   Seer chassis LiDARs
  /imu/chassis             Seer chassis IMU
  /scan/depth (+ /scan/top) depth-image-derived scans
  /scan/merged             all of the above merged, for SLAM / Nav2
  /camera/odom (+ /camera_top/odom) RGBD visual odometry

Usage:
  ros2 launch flexiv_amr_sensors sensors.launch.py
  ros2 launch flexiv_amr_sensors sensors.launch.py use_top_camera:=true
  ros2 launch flexiv_amr_sensors sensors.launch.py use_camera:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

AMR_IP = "192.168.1.110"
AMR_STATE_PORT = 19204

# Shared rtabmap rgbd_odometry tuning; frame/topics differ per camera.
_ODOM_PARAMS = {
    "publish_tf": False,
    "wait_for_transform": 0.2,
    "Odom/Strategy": "0",
    "Odom/ResetCountdown": "1",
    "OdomF2M/MaxSize": "1000",
    "Vis/CorNNType": "1",
    "Vis/MaxFeatures": "500",
    "Vis/MinInliers": "15",
}


def _realsense(camera_ns, serial, extra, condition):
    """rs_launch.py include. camera_ns=None keeps the default /camera namespace."""
    args = {
        "serial_no": serial,
        "initial_reset": "false",
        "enable_gyro": "true",
        "enable_accel": "true",
        "unite_imu_method": "1",
    }
    if camera_ns is not None:
        args["camera_name"] = camera_ns
        args["camera_namespace"] = camera_ns
    args.update(extra)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        launch_arguments=args.items(),
        condition=condition,
    )


def _depth_to_scan(name, base, output_frame, scan_topic, range_min, condition):
    return Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name=name,
        remappings=[
            ("depth", f"{base}/depth/image_rect_raw"),
            ("depth_camera_info", f"{base}/depth/camera_info"),
            ("scan", scan_topic),
        ],
        parameters=[
            {
                "output_frame": output_frame,
                "scan_height": 10,
                "range_min": range_min,
                "range_max": 10.0,
            }
        ],
        condition=condition,
    )


def _rgbd_odometry(name, base, frame_id, odom_frame, odom_topic, condition):
    return Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name=name,
        output="screen",
        parameters=[{"frame_id": frame_id, "odom_frame_id": odom_frame, **_ODOM_PARAMS}],
        remappings=[
            ("rgb/image", f"{base}/color/image_raw"),
            ("rgb/camera_info", f"{base}/color/camera_info"),
            ("depth/image", f"{base}/depth/image_rect_raw"),
            ("odom", odom_topic),
        ],
        condition=condition,
    )


def generate_launch_description():
    use_camera = LaunchConfiguration("use_camera")
    use_top_camera = LaunchConfiguration("use_top_camera")
    use_visual_odom = LaunchConfiguration("use_visual_odom")
    laserscan_topics = LaunchConfiguration("laserscan_topics")
    merger_delay = LaunchConfiguration("merger_delay")

    # Top camera actions need both their own flag and the master camera flag.
    top_camera_on = IfCondition(
        PythonExpression(["'", use_camera, "' == 'true' and '", use_top_camera, "' == 'true'"])
    )
    top_odom_on = IfCondition(
        PythonExpression(
            [
                "'", use_camera, "' == 'true' and '",
                use_top_camera, "' == 'true' and '",
                use_visual_odom, "' == 'true'",
            ]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_camera", default_value="true",
                              description="Launch RealSense camera(s) + depth-to-laserscan"),
        DeclareLaunchArgument("use_top_camera", default_value="false",
                              description="Also launch the head-mounted top D456 "
                                          "(adds /scan/top and a second visual odometry)"),
        DeclareLaunchArgument("use_visual_odom", default_value="true",
                              description="Launch RTAB-Map RGBD visual odometry"),
        DeclareLaunchArgument("camera_serial", default_value="_333422304124",
                              description="Bottom D456 serial number"),
        DeclareLaunchArgument("camera_top_serial", default_value="_324422301136",
                              description="Top (head) D456 serial number"),
        DeclareLaunchArgument("laserscan_topics",
                              default_value="/scan/nav /scan/avoid /scan/depth",
                              description="Space-separated scans to merge into "
                                          "/scan/merged (add /scan/top with the top camera)"),
        DeclareLaunchArgument("merger_delay", default_value="0.0",
                              description="Seconds to wait before starting the scan "
                                          "merger, so TF and the scans exist first"),

        # ===== Seer chassis sensors =====
        Node(
            package="flexiv_amr_driver",
            executable="seer_lidar_publisher",
            name="seer_lidar_publisher",
            output="screen",
            parameters=[{"amr_ip": AMR_IP, "amr_port": AMR_STATE_PORT, "poll_rate": 10.0}],
        ),
        Node(
            package="flexiv_amr_driver",
            executable="seer_imu_publisher",
            name="seer_imu_publisher",
            output="screen",
            parameters=[{"amr_ip": AMR_IP, "amr_port": AMR_STATE_PORT, "poll_rate": 50.0}],
        ),

        # ===== Bottom D456 (default /camera namespace) =====
        _realsense(None, LaunchConfiguration("camera_serial"), {}, IfCondition(use_camera)),
        _depth_to_scan(
            "depthimage_to_laserscan", "/camera/camera", "camera_depth_frame",
            "/scan/depth", 0.3, IfCondition(use_camera),
        ),
        _rgbd_odometry(
            "rgbd_odometry", "/camera/camera", "base_link", "odom_visual",
            "/camera/odom", IfCondition(use_visual_odom),
        ),

        # ===== Top D456 (head-mounted, forward-facing) =====
        _realsense(
            "camera_top",
            LaunchConfiguration("camera_top_serial"),
            {
                "base_frame_id": "camera_top_link",
                "publish_tf": "false",
                "enable_depth": "true",
                "enable_color": "true",
                "pointcloud.enable": "true",
                "depth_module.enable_auto_exposure": "true",
            },
            top_camera_on,
        ),
        _depth_to_scan(
            "depthimage_to_laserscan_top", "/camera_top/camera_top", "camera_top_link",
            "/scan/top", 0.4, top_camera_on,
        ),
        _rgbd_odometry(
            "rgbd_odometry_top", "/camera_top/camera_top", "camera_top_link",
            "odom_visual_top", "/camera_top/odom", top_odom_on,
        ),

        # ===== Scan merger -> /scan/merged for SLAM & Nav2 =====
        TimerAction(
            period=merger_delay,
            actions=[
                Node(
                    package="ira_laser_tools",
                    executable="laserscan_multi_merger",
                    name="laserscan_multi_merger",
                    output="screen",
                    parameters=[{
                        "destination_frame": "base_link",
                        "scan_destination_topic": "/scan/merged",
                        "laserscan_topics": laserscan_topics,
                        "angle_min": -3.14159,
                        "angle_max": 3.14159,
                        "range_min": 0.05,
                        "range_max": 50.0,
                    }],
                ),
            ],
        ),
    ])
