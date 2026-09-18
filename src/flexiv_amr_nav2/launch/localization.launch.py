from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('flexiv_amr_nav2')
    
    nav2_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'nav2_params.yaml'
    ])
    
    map_file = LaunchConfiguration('map')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('flexiv_amr_nav2'),
                'maps',
                'supermarket.yaml'
            ]),
            description='Full path to map yaml file'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename': map_file},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),
        
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                nav2_config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}
            ]
        ),
    ])