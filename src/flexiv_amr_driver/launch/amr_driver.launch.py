from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('flexiv_amr_driver')
    params_file = os.path.join(pkg_share, 'config', 'amr_params.yaml')
    
    return LaunchDescription([
        Node(
            package='flexiv_amr_driver',
            executable='velocity_controller',
            name='velocity_controller',
            output='screen',
            parameters=[params_file]
        ),
        
        Node(
            package='flexiv_amr_driver',
            executable='odometry_publisher',
            name='odometry_publisher',
            output='screen',
            parameters=[params_file]
        ),
        
        Node(
            package='flexiv_amr_driver',
            executable='status_monitor',
            name='status_monitor',
            output='screen',
            parameters=[params_file]
        ),
    ])