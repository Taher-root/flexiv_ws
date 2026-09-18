from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # Use SLAM Toolbox's built-in launch file (auto-activates)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ]),
            launch_arguments={
                'slam_params_file': PathJoinSubstitution([
                    FindPackageShare('flexiv_amr_nav2'),
                    'config',
                    'slam_toolbox.yaml'
                ]),
                'use_sim_time': 'false'
            }.items()
        )
    ])