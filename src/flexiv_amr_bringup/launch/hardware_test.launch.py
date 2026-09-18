from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    return LaunchDescription([
        # Launch AMR driver (velocity control, odometry, status)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('flexiv_amr_driver'),
                    'launch',
                    'amr_driver.launch.py'
                ])
            )
        ),
        
        # Launch sensors (camera, visual odometry, depth-to-laserscan)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('flexiv_amr_sensors'),
                    'launch',
                    'sensors.launch.py'
                ])
            )
        ),
        
        # Launch robot state publisher (URDF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('flexiv_amr_description'),
                    'launch',
                    'display.launch.py'
                ])
            ),
            launch_arguments={'use_gui': 'false'}.items()
        ),
    ])