#!/usr/bin/env python3
"""Robot hardware without mapping or navigation: URDF/TF, chassis, sensors.

AICO 2-4CE variant — uses the CE subnet (192.168.2.x).

Usage:
  ros2 launch flexiv_amr_bringup_ce hardware_test.launch.py
  ros2 launch flexiv_amr_bringup_ce hardware_test.launch.py use_robokit:=true
  ros2 launch flexiv_amr_bringup_ce hardware_test.launch.py use_camera:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

CE_AMR_IP = "192.168.2.110"


def _include(package, launch_file, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])
        ),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_robokit", default_value="false",
                              description="Use the Robokit chassis velocity controller"),
        DeclareLaunchArgument("use_camera", default_value="true",
                              description="Launch RealSense camera(s) + depth-to-laserscan"),
        DeclareLaunchArgument("use_top_camera", default_value="false",
                              description="Also launch the head-mounted top D456"),
        DeclareLaunchArgument("use_visual_odom", default_value="true",
                              description="Launch RTAB-Map RGBD visual odometry"),
        DeclareLaunchArgument("laserscan_topics",
                              default_value="/scan/nav /scan/avoid /scan/depth",
                              description="Scans merged into /scan/merged"),
        DeclareLaunchArgument("merger_delay", default_value="0.0",
                              description="Seconds to wait before starting the scan merger"),

        _include("flexiv_amr_description_ce", "display.launch.py", {"use_gui": "false"}),

        _include("flexiv_amr_driver", "amr_driver.launch.py", {
            "use_robokit": LaunchConfiguration("use_robokit"),
            "amr_ip": CE_AMR_IP,
        }),

        _include("flexiv_amr_sensors", "sensors.launch.py", {
            "amr_ip": CE_AMR_IP,
            "use_camera": LaunchConfiguration("use_camera"),
            "use_top_camera": LaunchConfiguration("use_top_camera"),
            "use_visual_odom": LaunchConfiguration("use_visual_odom"),
            "laserscan_topics": LaunchConfiguration("laserscan_topics"),
            "merger_delay": LaunchConfiguration("merger_delay"),
        }),
    ])
