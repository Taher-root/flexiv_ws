from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("rizon10s_driver"), "config", "arm.yaml"]
    )
    mock = LaunchConfiguration("mock_hardware")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mock_hardware",
                default_value="true",
                description="Simulate arm without flexivrdk",
            ),
            LifecycleNode(
                package="rizon10s_driver",
                executable="arm_driver",
                name="arm_driver",
                namespace="arm",
                output="screen",
                parameters=[config, {"mock_hardware": mock}],
            ),
        ]
    )
