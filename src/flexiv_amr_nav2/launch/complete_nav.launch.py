from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    mode = LaunchConfiguration('mode')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='mapping',
            description='Mode: mapping or navigation',
            choices=['mapping', 'navigation']
        ),
        
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('flexiv_amr_nav2'),
                'maps',
                'supermarket.yaml'
            ]),
            description='Map file for navigation mode'
        ),
        
        # Always launch EKF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('flexiv_amr_nav2'),
                    'launch',
                    'ekf.launch.py'
                ])
            )
        ),
        
        # Launch SLAM (mapping mode) or Localization (navigation mode)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('flexiv_amr_nav2'),
                    'launch',
                    'slam.launch.py'
                ])
            ),
            condition=IfCondition(PythonExpression(['"', mode, '" == "mapping"']))
        ),
        
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
            }.items(),
            condition=IfCondition(PythonExpression(['"', mode, '" == "navigation"']))
        ),
        
        # Always launch navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('flexiv_amr_nav2'),
                    'launch',
                    'navigation.launch.py'
                ])
            )
        ),
    ])