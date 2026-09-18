#!/usr/bin/env python3
"""Both Rizon arms (+ optional waist driver and joint state merger).

The single definition of the arm lifecycle nodes: their serials, joint names,
config files and configure/activate ordering. Composed by full_system.launch.py
and arms_with_navigation.launch.py rather than duplicated in each.

Usage:
  ros2 launch flexiv_amr_bringup arms.launch.py
  ros2 launch flexiv_amr_bringup arms.launch.py mock_hardware:=true
  ros2 launch flexiv_amr_bringup arms.launch.py enable_waist_driver:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, TimerAction
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

LEFT_JOINTS = [f"Left_joint{i}" for i in range(1, 8)]
RIGHT_JOINTS = [f"Right_joint{i}" for i in range(1, 8)]

LEFT_SN = "Rizon4-063352"
RIGHT_SN = "Rizon4R-062077"


def _lifecycle_timers(driver, configure_sec: float, activate_sec: float):
    """Timer actions that configure then activate a lifecycle node.

    A node whose own condition kept it from launching simply matches nothing
    here, so these stay harmless for the optional waist driver.
    """
    return [
        TimerAction(
            period=configure_sec,
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
    waist_pkg = FindPackageShare("aico2_waist_driver")

    mock_hardware = LaunchConfiguration("mock_hardware")
    enable_waist_driver = LaunchConfiguration("enable_waist_driver")
    use_merger = LaunchConfiguration("use_merger")

    left_driver = LifecycleNode(
        package="aico2_left_arm_driver",
        executable="left_arm_driver",
        name="left_arm_driver",
        namespace="left_arm",
        output="screen",
        parameters=[
            PathJoinSubstitution([left_pkg, "config", "left_arm_hardware.yaml"]),
            {
                "robot_sn": LEFT_SN,
                "auto_enable": True,
                "mock_hardware": mock_hardware,
                "joint_names": LEFT_JOINTS,
                "tcp_frame_id": "Left_flange",
            },
        ],
    )

    right_driver = LifecycleNode(
        package="aico2_right_arm_driver",
        executable="right_arm_driver",
        name="right_arm_driver",
        namespace="right_arm",
        output="screen",
        parameters=[
            PathJoinSubstitution([right_pkg, "config", "right_arm_hardware.yaml"]),
            {
                "robot_sn": RIGHT_SN,
                "auto_enable": True,
                "mock_hardware": mock_hardware,
                "joint_names": RIGHT_JOINTS,
                "tcp_frame_id": "Right_flange",
            },
        ],
    )

    # Waist axes (AGV_Jiont1/2): reads q[0:2] off the left arm's RDK stream via
    # its own session, independent of the arm drivers (see
    # docs/joint_state_architecture.md sec 3/4.2). Off by default: while
    # joint_state_merger is still publishing those two joints as 0.0, enabling
    # this races last-writer-wins on them (sec 8 step 3 — intentional during
    # verification, not the steady state).
    waist_driver = LifecycleNode(
        package="aico2_waist_driver",
        executable="waist_driver",
        name="waist_driver",
        output="screen",
        condition=IfCondition(enable_waist_driver),
        parameters=[
            PathJoinSubstitution([waist_pkg, "config", "waist_driver.yaml"]),
            {"mock_hardware": mock_hardware},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mock_hardware",
                default_value="false",
                description="Simulate the arms without connecting via flexivrdk",
            ),
            DeclareLaunchArgument(
                "enable_waist_driver",
                default_value="false",
                description="Publish real AGV_Jiont1/2 from the RDK stream "
                            "(races joint_state_merger's zeros; see "
                            "aico2_waist_driver/README.md)",
            ),
            DeclareLaunchArgument(
                "use_merger",
                default_value="true",
                description="Merge /left_arm + /right_arm joint_states into "
                            "/joint_states (16 joints, waist at 0.0)",
            ),
            left_driver,
            right_driver,
            waist_driver,
            *_lifecycle_timers(left_driver, 1.0, 3.0),
            *_lifecycle_timers(right_driver, 1.0, 4.0),
            # Waist configures after the left arm it reads from — no hard
            # dependency, just avoiding a burst of simultaneous RDK connects.
            *_lifecycle_timers(waist_driver, 3.5, 5.0),
            Node(
                package="flexiv_amr_driver",
                executable="joint_state_merger",
                name="joint_state_merger",
                output="screen",
                condition=IfCondition(use_merger),
                parameters=[
                    {
                        "left_arm_topic": "/left_arm/joint_states",
                        "right_arm_topic": "/right_arm/joint_states",
                        "rate_hz": 50.0,
                    }
                ],
            ),
        ]
    )
