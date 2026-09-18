#!/usr/bin/env python3
"""Standalone launch for the waist driver (configure + activate).

Transitions are event-driven rather than on blind wall-clock timers: CONFIGURE
fires a grace period after the process itself starts (its lifecycle service is
not discoverable the instant the process spawns), and ACTIVATE waits for that
configure to actually succeed. See flexiv_amr_bringup/launch/arms.launch.py for
the failure that motivated this.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

_SERVICE_GRACE_SEC = 2.0


def generate_launch_description():
    pkg = FindPackageShare("aico2_waist_driver")
    mock_hardware = LaunchConfiguration("mock_hardware")

    waist_driver = LifecycleNode(
        package="aico2_waist_driver",
        executable="waist_driver",
        name="waist_driver",
        namespace="",
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
            RegisterEventHandler(
                OnProcessStart(
                    target_action=waist_driver,
                    on_start=[
                        TimerAction(
                            period=_SERVICE_GRACE_SEC,
                            actions=[
                                EmitEvent(
                                    event=ChangeState(
                                        lifecycle_node_matcher=matches_action(
                                            waist_driver
                                        ),
                                        transition_id=Transition.TRANSITION_CONFIGURE,
                                    )
                                )
                            ],
                        )
                    ],
                )
            ),
            RegisterEventHandler(
                OnStateTransition(
                    target_lifecycle_node=waist_driver,
                    # Pinned to a successful configure so a later deactivate
                    # does not silently re-activate the node.
                    start_state="configuring",
                    goal_state="inactive",
                    entities=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(waist_driver),
                                transition_id=Transition.TRANSITION_ACTIVATE,
                            )
                        )
                    ],
                )
            ),
        ]
    )
