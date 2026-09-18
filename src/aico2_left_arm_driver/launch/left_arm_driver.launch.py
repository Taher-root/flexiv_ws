from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("aico2_left_arm_driver"), "config", "left_arm.yaml"]
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
                package="aico2_left_arm_driver",
                executable="left_arm_driver",
                name="left_arm_driver",
                namespace="left_arm",
                output="screen",
                parameters=[config, {"mock_hardware": mock}],
            ),
        ]
    )
