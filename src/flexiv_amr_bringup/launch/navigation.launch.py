from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('flexiv_amr_nav2'),
                'maps',
                'map.yaml'
            ]),
            description='Full path to map yaml file'
        ),
        
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
        
        # Launch localization (AMCL + map server)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('flexiv_amr_nav2'),
                    'launch',
                    'localization.launch.py'
                ])
            ),
            launch_arguments={
                'map': LaunchConfiguration('map')
            }.items()
        ),
        
        # Launch NAV2 stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('flexiv_amr_nav2'),
                    'launch',
                    'navigation.launch.py'
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
                'navigation.rviz'
            ])],
            condition=launch.conditions.IfCondition(LaunchConfiguration('use_rviz'))
        ),
    ])