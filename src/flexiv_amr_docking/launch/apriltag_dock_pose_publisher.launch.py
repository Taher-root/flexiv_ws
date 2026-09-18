from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    apriltag_config = os.path.join(
        get_package_share_directory('flexiv_amr_docking'),
        'config',
        'apriltags_36h11.yaml'
    )

    nav2_params = os.path.join(
        get_package_share_directory('flexiv_amr_nav2'),
        'config',
        'nav2_params.yaml'
    )

    return LaunchDescription([
        # Step 1: Rectify raw color image
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_color',
            namespace='camera/camera/color',
            remappings=[
                ('image',      'image_raw'),
                ('camera_info','camera_info'),
                ('image_rect', 'image_rect'),
            ],
            parameters=[{
                'qos_overrides./camera/camera/color/camera_info.subscription.durability': 'volatile'
            }],
        ),

        # Step 2: Detect AprilTags
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            namespace='',
            remappings=[
                ('image_rect',  '/camera/camera/color/image_rect'),
                ('camera_info', '/camera/camera/color/camera_info'),
            ],
            parameters=[apriltag_config],
        ),

        # Step 3: Bridge node - TF -> /detected_dock_pose
        Node(
            package='flexiv_amr_docking',
            executable='detected_dock_pose_publisher.py',
            name='detected_dock_pose_publisher',
            output='screen',
        ),
    ])
