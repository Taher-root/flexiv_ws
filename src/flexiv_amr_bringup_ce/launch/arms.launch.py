#!/usr/bin/env python3
"""Both Rizon arms (+ optional waist driver and joint state merger).

AICO 2-4CE variant (software v3.11.2 CE, RDK v1.9.3).

Key differences from the original AICO2:
  - Different robot serial numbers
  - Base key switch safety mode (manual = debug mode)
  - Lidar safeguard stop: arms get a safeguard stop if an obstruction
    gets too close to the base while running in automatic mode
  - Wheel motor brakes (require power-on + e-stop + release button to push)

Usage:
  ros2 launch flexiv_amr_bringup_ce arms.launch.py
  ros2 launch flexiv_amr_bringup_ce arms.launch.py mock_hardware:=true
  ros2 launch flexiv_amr_bringup_ce arms.launch.py enable_waist_driver:=true
  ros2 launch flexiv_amr_bringup_ce arms.launch.py teleop_backend:=servo
  ros2 launch flexiv_amr_bringup_ce arms.launch.py \
      joint_control_mode:=impedance joint_stiffness_ratio:=0.15 \
      max_contact_torque:=10.0
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

LEFT_JOINTS = [f"Left_joint{i}" for i in range(1, 8)]
RIGHT_JOINTS = [f"Right_joint{i}" for i in range(1, 8)]

LEFT_SN = "REPLACE_WITH_CE_LEFT_SERIAL"
RIGHT_SN = "REPLACE_WITH_CE_RIGHT_SERIAL"

_SERVICE_GRACE_SEC = 2.0


def _lifecycle_startup(driver, connect_delay_sec: float = 0.0):
    """Configure a lifecycle node once it is up, then activate it once
    configure has actually succeeded.
    """
    return [
        RegisterEventHandler(
            OnProcessStart(
                target_action=driver,
                on_start=[
                    TimerAction(
                        period=_SERVICE_GRACE_SEC + connect_delay_sec,
                        actions=[
                            EmitEvent(
                                event=ChangeState(
                                    lifecycle_node_matcher=matches_action(driver),
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
                target_lifecycle_node=driver,
                start_state="configuring",
                goal_state="inactive",
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(driver),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )
                    )
                ],
            )
        ),
    ]


def _build_nodes(context):
    ce_pkg = FindPackageShare("flexiv_amr_bringup_ce")
    waist_pkg = FindPackageShare("aico2_waist_driver")

    mock_hardware = LaunchConfiguration("mock_hardware")
    enable_waist_driver = LaunchConfiguration("enable_waist_driver")
    joint_control_mode = ParameterValue(
        LaunchConfiguration("joint_control_mode"), value_type=str)
    joint_stiffness_ratio = ParameterValue(
        LaunchConfiguration("joint_stiffness_ratio"), value_type=float)
    max_contact_torque = ParameterValue(
        LaunchConfiguration("max_contact_torque"), value_type=float)
    control_waist = ParameterValue(
        LaunchConfiguration("control_waist"), value_type=bool)

    teleop_backend = LaunchConfiguration("teleop_backend").perform(context)
    backend_override = {"teleop_backend": teleop_backend} if teleop_backend else {}

    joint_states_remap = [("joint_states", "joint_states")]

    left_driver = LifecycleNode(
        package="aico2_left_arm_driver",
        executable="left_arm_driver",
        name="left_arm_driver",
        namespace="left_arm",
        output="screen",
        remappings=joint_states_remap,
        parameters=[
            PathJoinSubstitution([ce_pkg, "config", "left_arm_hardware.yaml"]),
            {
                "robot_sn": LEFT_SN,
                "auto_enable": True,
                "mock_hardware": mock_hardware,
                "joint_names": LEFT_JOINTS,
                "tcp_frame_id": "Left_flange",
                "joint_control_mode": joint_control_mode,
                "joint_stiffness_ratio": joint_stiffness_ratio,
                "max_contact_torque": max_contact_torque,
                "control_waist": control_waist,
                **backend_override,
            },
        ],
    )

    right_driver = LifecycleNode(
        package="aico2_right_arm_driver",
        executable="right_arm_driver",
        name="right_arm_driver",
        namespace="right_arm",
        output="screen",
        remappings=joint_states_remap,
        parameters=[
            PathJoinSubstitution([ce_pkg, "config", "right_arm_hardware.yaml"]),
            {
                "robot_sn": RIGHT_SN,
                "auto_enable": True,
                "mock_hardware": mock_hardware,
                "joint_names": RIGHT_JOINTS,
                "tcp_frame_id": "Right_flange",
                "joint_control_mode": joint_control_mode,
                "joint_stiffness_ratio": joint_stiffness_ratio,
                "max_contact_torque": max_contact_torque,
                **backend_override,
            },
        ],
    )

    waist_driver = LifecycleNode(
        package="aico2_waist_driver",
        executable="waist_driver",
        name="waist_driver",
        namespace="",
        output="screen",
        condition=IfCondition(enable_waist_driver),
        parameters=[
            PathJoinSubstitution([waist_pkg, "config", "waist_driver.yaml"]),
            {"mock_hardware": mock_hardware},
        ],
    )

    return [
        left_driver,
        right_driver,
        waist_driver,
        *_lifecycle_startup(left_driver),
        *_lifecycle_startup(right_driver, connect_delay_sec=1.5),
        *_lifecycle_startup(waist_driver, connect_delay_sec=3.0),
        Node(
            package="flexiv_amr_driver",
            executable="joint_state_merger",
            name="joint_state_merger",
            output="screen",
            parameters=[
                {
                    "left_arm_topic": "/left_arm/joint_states",
                    "right_arm_topic": "/right_arm/joint_states",
                    "rate_hz": 50.0,
                }
            ],
        ),
    ]


def generate_launch_description():
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
                description="Publish real AGV_Joint1/2 from the RDK stream",
            ),
            DeclareLaunchArgument(
                "joint_control_mode",
                default_value="position",
                description="position: NRT_JOINT_POSITION. impedance: "
                            "NRT_JOINT_IMPEDANCE, compliant",
            ),
            DeclareLaunchArgument(
                "joint_stiffness_ratio",
                default_value="1.0",
                description="fraction of K_q_nom when "
                            "joint_control_mode:=impedance",
            ),
            DeclareLaunchArgument(
                "control_waist",
                default_value="false",
                description="true: LEFT arm driver owns AGV_Joint1/2",
            ),
            DeclareLaunchArgument(
                "max_contact_torque",
                default_value="0.0",
                description="Nm ceiling on torque per axis in impedance mode; "
                            "0 leaves unbounded",
            ),
            DeclareLaunchArgument(
                "teleop_backend",
                default_value="",
                description="Override teleop_backend for both arm drivers. "
                            "Empty uses each driver's config file value. "
                            "'servo' for IK-based VR teleop, 'cartesian' for "
                            "twist-based control.",
            ),
            OpaqueFunction(function=_build_nodes),
        ]
    )
