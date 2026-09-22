#!/usr/bin/env python3
"""Both Rizon arms (+ optional waist driver and joint state merger).

The single definition of the arm lifecycle nodes: their serials, joint names,
config files and configure/activate ordering. Composed by full_system.launch.py
and arms_with_navigation.launch.py rather than duplicated in each.

Usage:
  ros2 launch flexiv_amr_bringup arms.launch.py
  ros2 launch flexiv_amr_bringup arms.launch.py mock_hardware:=true
  ros2 launch flexiv_amr_bringup arms.launch.py enable_waist_driver:=true
  ros2 launch flexiv_amr_bringup arms.launch.py \
      joint_control_mode:=impedance joint_stiffness_ratio:=0.15 \
      max_contact_torque:=10.0
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
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

LEFT_SN = "Rizon4-063352"
RIGHT_SN = "Rizon4R-062077"

# Grace period between a driver process spawning and its lifecycle service
# being discoverable. Blind wall-clock timers from launch start are not
# enough: on 2026-09-18 the left arm missed a CONFIGURE emitted 1.0s after
# launch (its service wasn't up yet), then died when ACTIVATE arrived 2s
# later against a still-unconfigured state machine. Timed from each
# process's own start instead, and ACTIVATE now waits for configure to
# actually succeed rather than guessing how long it takes.
_SERVICE_GRACE_SEC = 2.0


def _lifecycle_startup(driver, connect_delay_sec: float = 0.0):
    """Configure a lifecycle node once it is up, then activate it once
    configure has actually succeeded.

    connect_delay_sec staggers CONFIGURE across drivers — that transition is
    where the blocking flexivrdk connect happens (~1.35s measured), so
    spreading it avoids several simultaneous connects to the controllers.

    A node whose own condition kept it from launching never emits
    OnProcessStart, so these handlers stay harmless for the optional waist
    driver.
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
                # start_state pins this to a successful configure, so a later
                # deactivate (active -> inactive) does not silently re-activate.
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


def generate_launch_description():
    left_pkg = FindPackageShare("aico2_left_arm_driver")
    right_pkg = FindPackageShare("aico2_right_arm_driver")
    waist_pkg = FindPackageShare("aico2_waist_driver")

    mock_hardware = LaunchConfiguration("mock_hardware")
    enable_waist_driver = LaunchConfiguration("enable_waist_driver")
    direct_joint_states = LaunchConfiguration("direct_joint_states")
    joint_control_mode = ParameterValue(
        LaunchConfiguration("joint_control_mode"), value_type=str)
    # Without value_type a LaunchConfiguration arrives as a string and the
    # driver's double parameter rejects it as a type mismatch.
    joint_stiffness_ratio = ParameterValue(
        LaunchConfiguration("joint_stiffness_ratio"), value_type=float)
    max_contact_torque = ParameterValue(
        LaunchConfiguration("max_contact_torque"), value_type=float)

    # joint_state_architecture.md sec 3 / sec 8 step 6: the target state is
    # every driver publishing its own joints straight to /joint_states, with
    # robot_state_publisher merging the partial messages by name and no merger
    # in the middle. Done as a remap rather than a code change so the cutover
    # is one launch argument and reverting is instant — nothing in this repo
    # consumes /left_arm/joint_states except the merger, but the VR teleop and
    # the other machines on this ROS domain are outside it and unverified.
    #   false (default): "joint_states" -> /<ns>/joint_states, merger stitches
    #   true:            "joint_states" -> /joint_states, merger not started
    joint_states_remap = [(
        "joint_states",
        PythonExpression([
            "'/joint_states' if '", direct_joint_states,
            "' == 'true' else 'joint_states'",
        ]),
    )]

    left_driver = LifecycleNode(
        package="aico2_left_arm_driver",
        executable="left_arm_driver",
        name="left_arm_driver",
        namespace="left_arm",
        output="screen",
        remappings=joint_states_remap,
        parameters=[
            PathJoinSubstitution([left_pkg, "config", "left_arm_hardware.yaml"]),
            {
                "robot_sn": LEFT_SN,
                "auto_enable": True,
                "mock_hardware": mock_hardware,
                "joint_names": LEFT_JOINTS,
                "tcp_frame_id": "Left_flange",
                # Overridable here because joint_control_mode is only read in
                # on_configure: changing it later needs a relaunch, so it has
                # to be settable without editing the YAML.
                # joint_stiffness_ratio is also a live parameter:
                #   ros2 param set /left_arm/left_arm_driver \
                #       joint_stiffness_ratio 0.3
                "joint_control_mode": joint_control_mode,
                "joint_stiffness_ratio": joint_stiffness_ratio,
                "max_contact_torque": max_contact_torque,
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
            PathJoinSubstitution([right_pkg, "config", "right_arm_hardware.yaml"]),
            {
                "robot_sn": RIGHT_SN,
                "auto_enable": True,
                "mock_hardware": mock_hardware,
                "joint_names": RIGHT_JOINTS,
                "tcp_frame_id": "Right_flange",
                "joint_control_mode": joint_control_mode,
                "joint_stiffness_ratio": joint_stiffness_ratio,
                "max_contact_torque": max_contact_torque,
            },
        ],
    )

    # Waist axes (AGV_Joint1/2): reads q[0:2] off the left arm's RDK stream via
    # its own session, independent of the arm drivers (see
    # docs/joint_state_architecture.md sec 3/4.2). Off by default: while
    # joint_state_merger is still publishing those two joints as 0.0, enabling
    # this races last-writer-wins on them (sec 8 step 3 — intentional during
    # verification, not the steady state).
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
                description="Publish real AGV_Joint1/2 from the RDK stream "
                            "(races joint_state_merger's zeros; see "
                            "aico2_waist_driver/README.md)",
            ),
            DeclareLaunchArgument(
                "direct_joint_states",
                default_value="false",
                description="false: arms publish /<ns>/joint_states and "
                            "joint_state_merger stitches them into /joint_states "
                            "(16 joints, waist forced to 0.0). true: arms publish "
                            "their own joints straight to /joint_states and the "
                            "merger is not started — robot_state_publisher merges "
                            "by joint name, so the waist driver's real values are "
                            "no longer overwritten with zeros",
            ),
            DeclareLaunchArgument(
                "joint_control_mode",
                default_value="position",
                description="position: NRT_JOINT_POSITION, a stiff position "
                            "controller (historical behaviour). impedance: "
                            "NRT_JOINT_IMPEDANCE, tracking the same "
                            "SendJointPosition stream but yielding to external "
                            "force at joint_stiffness_ratio x K_q_nom. Applies "
                            "to both trajectories and servo teleop, and is read "
                            "once in on_configure — hence a launch argument "
                            "rather than something to set at runtime",
            ),
            DeclareLaunchArgument(
                "joint_stiffness_ratio",
                default_value="1.0",
                description="fraction of K_q_nom on the arm axes when "
                            "joint_control_mode:=impedance (the waist entries "
                            "are +inf nominal and pass through untouched). "
                            "Retunable live: ros2 param set "
                            "/left_arm/left_arm_driver joint_stiffness_ratio 0.3",
            ),
            DeclareLaunchArgument(
                "max_contact_torque",
                default_value="0.0",
                description="Nm ceiling on the torque each arm axis will apply "
                            "against the environment in impedance mode, clamped "
                            "per axis to tau_max. Without it the impedance law "
                            "demands stiffness x deflection unbounded, so a "
                            "blocked arm pushes until the controller's collision "
                            "detection faults it -- which is what a compliance "
                            "test looks like when it goes wrong. 10 Nm is "
                            "roughly 25 N at the forearm. 0 leaves it unset. "
                            "Retunable live: ros2 param set "
                            "/left_arm/left_arm_driver max_contact_torque 10.0",
            ),
            left_driver,
            right_driver,
            waist_driver,
            # Staggered so the blocking RDK connects in each CONFIGURE do not
            # overlap; each ACTIVATE then follows its own successful configure.
            *_lifecycle_startup(left_driver),
            *_lifecycle_startup(right_driver, connect_delay_sec=1.5),
            # Waist reads the left arm's controller — no hard ordering
            # dependency, it just goes last.
            *_lifecycle_startup(waist_driver, connect_delay_sec=3.0),
            Node(
                package="flexiv_amr_driver",
                executable="joint_state_merger",
                name="joint_state_merger",
                output="screen",
                condition=UnlessCondition(direct_joint_states),
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
