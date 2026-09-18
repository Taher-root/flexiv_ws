from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for visualization'
        ),

        # Launch hardware (driver + sensors + URDF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('flexiv_amr_bringup'),
                    'launch',
                    'hardware_test.launch.py'
                ])
            )
        ),

        # Launch EKF (sensor fusion)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('flexiv_amr_nav2'),
                    'launch',
                    'ekf.launch.py'
                ])
            )
        ),

        # Launch SLAM Toolbox (mapping)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('flexiv_amr_nav2'),
                    'launch',
                    'slam.launch.py'
                ])
            )
        ),

        # Launch RViz (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('flexiv_amr_bringup'),
                'rviz',
                'mapping.rviz'
            ])],
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        ),
    ])

