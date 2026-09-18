#!/usr/bin/env python3
"""Launch arms + joint state merger + navigation stack"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from lifecycle_msgs.msg import Transition

LEFT_JOINTS = [f"Left_joint{i}" for i in range(1, 8)]
RIGHT_JOINTS = [f"Right_joint{i}" for i in range(1, 8)]

def _lifecycle_timers(driver, activate_sec: float):
    return [
        TimerAction(
            period=1.0,
            actions=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(driver),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                )
            ],
        ),
        TimerAction(
            period=activate_sec,
            actions=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(driver),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        ),
    ]

def generate_launch_description():
    left_pkg = FindPackageShare("aico2_left_arm_driver")
    right_pkg = FindPackageShare("aico2_right_arm_driver")

    # Left arm driver
    left_driver = LifecycleNode(
        package="aico2_left_arm_driver",
        executable="left_arm_driver",
        name="left_arm_driver",
        namespace="left_arm",
        output="screen",
        parameters=[
            PathJoinSubstitution([left_pkg, "config", "left_arm_hardware.yaml"]),
            {
                "robot_sn": "Rizon4-063352",
                "auto_enable": True,
                "mock_hardware": False,
                "joint_names": LEFT_JOINTS,
                "tcp_frame_id": "Left_flange",
            },
        ],
    )

    # Right arm driver
    right_driver = LifecycleNode(
        package="aico2_right_arm_driver",
        executable="right_arm_driver",
        name="right_arm_driver",
        namespace="right_arm",
        output="screen",
        parameters=[
            PathJoinSubstitution([right_pkg, "config", "right_arm_hardware.yaml"]),
            {
                "robot_sn": "Rizon4R-062077",
                "auto_enable": True,
                "mock_hardware": False,
                "joint_names": RIGHT_JOINTS,
                "tcp_frame_id": "Right_flange",
            },
        ],
    )

    # Joint state merger
    joint_merger = Node(
        package="flexiv_amr_driver",
        executable="joint_state_merger",
        name="joint_state_merger",
        output="screen",
        parameters=[{
            "left_arm_topic": "/left_arm/joint_states",
            "right_arm_topic": "/right_arm/joint_states",
            "rate_hz": 50.0,
        }],
    )

    # Include the rest of the navigation stack
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('flexiv_amr_bringup'),
                'launch',
                'mapping_robokit.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        left_driver,
        right_driver,
        *_lifecycle_timers(left_driver, 3.0),
        *_lifecycle_timers(right_driver, 4.0),
        joint_merger,
        TimerAction(
            period=6.0,  # Wait for arms to be ready
            actions=[nav_launch]
        ),
    ])
