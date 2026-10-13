from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mock = LaunchConfiguration("mock_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    robot_sn = LaunchConfiguration("robot_sn")

    config_file = LaunchConfiguration("config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mock_hardware",
                default_value="true",
                description="Simulate arm without flexivrdk",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="192.168.100.1",
                description="IP address of the Rizon 10s arm",
            ),
            DeclareLaunchArgument(
                "robot_sn",
                default_value="Rizon10s-062545",
                description="Serial number of the Rizon 10s arm",
            ),
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("rizon10s_driver"), "config", "arm.yaml"]
                ),
                description="Path to the driver config YAML",
            ),
            LifecycleNode(
                package="rizon10s_driver",
                executable="arm_driver",
                name="arm_driver",
                namespace="arm",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "mock_hardware": mock,
                        "robot_ip": robot_ip,
                        "robot_sn": robot_sn,
                    },
                ],
            ),
        ]
    )
