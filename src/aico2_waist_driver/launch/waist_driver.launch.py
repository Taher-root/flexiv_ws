#!/usr/bin/env python3
"""Standalone launch for the waist driver (configure + activate)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, TimerAction
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg = FindPackageShare("aico2_waist_driver")
    mock_hardware = LaunchConfiguration("mock_hardware")

    waist_driver = LifecycleNode(
        package="aico2_waist_driver",
        executable="waist_driver",
        name="waist_driver",
        output="screen",
        parameters=[
            PathJoinSubstitution([pkg, "config", "waist_driver.yaml"]),
            {"mock_hardware": mock_hardware},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mock_hardware",
                default_value="true",
                description="Use mock hardware (no RDK connection)",
            ),
            waist_driver,
            TimerAction(
                period=1.0,
                actions=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(waist_driver),
                            transition_id=Transition.TRANSITION_CONFIGURE,
                        )
                    )
                ],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(waist_driver),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )
                    )
                ],
            ),
        ]
    )
