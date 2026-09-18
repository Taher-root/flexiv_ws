from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get URDF file path
    pkg_share = get_package_share_directory('flexiv_amr_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'AICO2-Rizon4.urdf')
    
    # Read URDF file content
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()
    
    # Wrap as ParameterValue with explicit string type
    robot_description = ParameterValue(robot_description_content, value_type=str)
    
    use_gui = LaunchConfiguration('use_gui')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui',
            default_value='false',
            description='Launch joint_state_publisher_gui'
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 50.0,
            }]
        ),
        
        # Joint State Publisher GUI (only if use_gui=true)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(use_gui)
        ),
        
        # RViz removed - launched separately in mapping_robokit.launch.py
    ])
