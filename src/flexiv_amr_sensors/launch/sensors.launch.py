from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_visual_odom = LaunchConfiguration('use_visual_odom')
    use_camera = LaunchConfiguration('use_camera')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_visual_odom',
            default_value='true',
            description='Use visual odometry'
        ),

        DeclareLaunchArgument(
            'use_camera',
            default_value='true',
            description='Launch RealSense camera and depth-to-laserscan'
        ),

        # ===== Seer Chassis Sensors =====
        # LiDAR publisher (front /scan/nav + rear /scan/avoid)
        Node(
            package='flexiv_amr_driver',
            executable='seer_lidar_publisher',
            name='seer_lidar_publisher',
            output='screen',
            parameters=[{
                'amr_ip': '192.168.1.110',
                'amr_port': 19204,
                'poll_rate': 10.0,
            }]
        ),

        # IMU publisher (/imu/chassis)
        Node(
            package='flexiv_amr_driver',
            executable='seer_imu_publisher',
            name='seer_imu_publisher',
            output='screen',
            parameters=[{
                'amr_ip': '192.168.1.110',
                'amr_port': 19204,
                'poll_rate': 50.0,
            }]
        ),

        # ===== RealSense Camera (conditional) =====
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            ),
            launch_arguments={
                'initial_reset': 'false',
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': '1',
            }.items(),
            condition=IfCondition(use_camera)
        ),

        # Depth to laserscan -> publishes /scan/depth
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[
                ('depth', '/camera/camera/depth/image_rect_raw'),
                ('depth_camera_info', '/camera/camera/depth/camera_info'),
                ('scan', '/scan/depth')
            ],
            parameters=[{
                'output_frame': 'camera_depth_frame',
                'scan_height': 10,
                'range_min': 0.3,
                'range_max': 10.0,
            }],
            condition=IfCondition(use_camera)
        ),

        # ===== Laser Scan Merger (ira_laser_tools) =====
        # Merges all scan sources -> /scan/merged for SLAM & Nav2
        Node(
            package='ira_laser_tools',
            executable='laserscan_multi_merger',
            name='laserscan_multi_merger',
            output='screen',
            parameters=[{
                'destination_frame': 'base_link',
                'scan_destination_topic': '/scan/merged',
                'laserscan_topics': '/scan/nav /scan/avoid /scan/depth',
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'range_min': 0.05,
                'range_max': 50.0,
            }]
        ),

        # ===== Visual Odometry (conditional) =====
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom_visual',
                'publish_tf': False,
                'wait_for_transform': 0.2,
                'Odom/Strategy': '0',
                'Odom/ResetCountdown': '1',
                'OdomF2M/MaxSize': '1000',
                'Vis/CorNNType': '1',
                'Vis/MaxFeatures': '500',
                'Vis/MinInliers': '15',
            }],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/depth/image_rect_raw'),
                ('odom', '/camera/odom')
            ],
            condition=IfCondition(use_visual_odom)
        ),
    ])
