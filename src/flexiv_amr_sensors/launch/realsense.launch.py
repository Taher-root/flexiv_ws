from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument('enable_depth', default_value='true'),
        DeclareLaunchArgument('enable_color', default_value='true'),
        DeclareLaunchArgument('enable_infra', default_value='false'),
        DeclareLaunchArgument('enable_gyro', default_value='true'),
        DeclareLaunchArgument('enable_accel', default_value='true'),
        DeclareLaunchArgument('unite_imu_method', default_value='linear_interpolation'),
        
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ]),
            launch_arguments={
                'enable_depth': LaunchConfiguration('enable_depth'),
                'enable_color': LaunchConfiguration('enable_color'),
                'enable_infra': LaunchConfiguration('enable_infra'),
                'enable_gyro': LaunchConfiguration('enable_gyro'),
                'enable_accel': LaunchConfiguration('enable_accel'),
                'unite_imu_method': LaunchConfiguration('unite_imu_method'),
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile': '640x480x30',
                'align_depth.enable': 'true',
                'pointcloud.enable': 'false',
            }.items()
        ),
        
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[
                ('depth', '/camera/depth/image_rect_raw'),
                ('depth_camera_info', '/camera/depth/camera_info'),
                ('scan', '/scan')
            ],
            parameters=[{
                'output_frame': 'camera_depth_frame',
                'scan_height': 10,
                'range_min': 0.3,
                'range_max': 10.0,
            }]
        ),
    ])