"""VR teleop session: cameras + video streaming + MoveIt Servo + RT bridge.

One command brings up everything the operator needs for VR teleop with live
camera feeds, collision-checked arm motion, and the 1 kHz RT control loop:

  ros2 launch flexiv_amr_bringup teleop.launch.py use_vr:=true

What runs (in staged order):

  1. robot_state_publisher            URDF -> TF
  2. rt_bridge (ns /<arm>_arm)        shm <-> ROS (joint_states, servo, FJT)
  3. RealSense D456 cameras            /cam_left, /cam_right, /cam_head
  4. foxglove_bridge                   WebSocket -> Foxglove Studio in browser
  5. move_group                        planning scene for collision checking
  6. servo_node                        TwistStamped -> JointTrajectory at 100 Hz
  7. vr_bridge (opt)                   WebXR poses -> Servo twist at port 8181

rt_server must already be running in a separate (ROS-free) terminal.
With arm:=both, two rt_server instances are needed (one per arm, each with
its own --shm name matching shm_name_left / shm_name_right).

Camera serials are baked in as defaults (left: 327622300610, right: 327622300144,
head: 324422301136). Override with cam_left_serial, cam_right_serial,
cam_head_serial if you swap cameras.

Video streaming: open https://app.foxglove.dev in a browser and connect to
ws://<laptop-ip>:8765. Add Image panels for /cam_left/cam_left/color/image_raw,
/cam_right/cam_right/color/image_raw, /cam_head/cam_head/color/image_raw.

Usage:
  # Default (left arm, no VR):
  ros2 launch flexiv_amr_bringup teleop.launch.py

  # Right arm with VR teleop:
  ros2 launch flexiv_amr_bringup teleop.launch.py arm:=right use_vr:=true

  # Both arms with VR teleop:
  ros2 launch flexiv_amr_bringup teleop.launch.py arm:=both use_vr:=true

  # With episode recording for pi0.5 data collection:
  ros2 launch flexiv_amr_bringup teleop.launch.py arm:=left use_vr:=true use_recorder:=true

  # Skip cameras (already running elsewhere):
  ros2 launch flexiv_amr_bringup teleop.launch.py use_cameras:=false

  # Skip Servo (direct joint streaming from VR IK):
  ros2 launch flexiv_amr_bringup teleop.launch.py use_servo:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_PLANNING_FRAMES = {"left": "Left_link0", "right": "Right_link0"}
_DEFAULT_SHM = {
    "left": "/aico2_rt_control",
    "right": "/aico2_rt_control_right",
}

SERVO_DELAY = 4.0
MOVE_GROUP_DELAY = 2.0
VR_DELAY = 5.0


def _camera(camera_name, serial, condition):
    """One RealSense D456: color only, no depth-to-laserscan, no pointcloud."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"), "launch", "rs_launch.py",
            ])
        ),
        launch_arguments={
            "camera_name": camera_name,
            "camera_namespace": camera_name,
            "serial_no": serial,
            "initial_reset": "false",
            "enable_color": "true",
            "enable_depth": "false",
            "enable_infra": "false",
            "enable_gyro": "false",
            "enable_accel": "false",
            "rgb_camera.profile": "640x480x30",
        }.items(),
        condition=condition,
    )


def _include(package, launch_file, launch_arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])
        ),
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )


def _arm_nodes(arm, shm_name, servo_on, vr_on, include_vr=True):
    """Nodes for one arm: rt_bridge + servo + optionally vr_bridge."""
    arm_ns = f"{arm}_arm"
    planning_frame = _PLANNING_FRAMES.get(arm, "Left_link0")
    servo_node_name = f"servo_node_{arm}"
    twist_topic = f"/{servo_node_name}/delta_twist_cmds"
    prefix = arm.capitalize()
    joint_names = (
        ["AGV_Joint1", "AGV_Joint2"]
        + [f"{prefix}_joint{i}" for i in range(1, 8)]
    )
    servo_joint_names = [f"{prefix}_joint{i}" for i in range(1, 8)]
    nodes = [
        Node(
            package="aico2_rt_control",
            executable="rt_bridge",
            name="rt_bridge",
            namespace=arm_ns,
            output="screen",
            parameters=[{
                "shm_name": shm_name,
                "joint_names": joint_names,
                "servo_joint_names": servo_joint_names,
            }],
        ),
        TimerAction(
            period=SERVO_DELAY,
            actions=[
                _include("aico2_moveit_config", "servo.launch.py",
                         launch_arguments={
                             "arm": arm,
                             "node_name": servo_node_name,
                         },
                         condition=servo_on),
            ],
        ),
    ]
    if include_vr:
        nodes.append(
            TimerAction(
                period=VR_DELAY,
                actions=[
                    Node(
                        package="aico2_vr_teleop",
                        executable="vr_bridge",
                        name=f"vr_bridge_{arm}",
                        output="screen",
                        parameters=[{
                            "hand": arm,
                            "planning_frame": planning_frame,
                            "arm_ns": arm_ns,
                            "twist_topic": twist_topic,
                        }],
                        condition=vr_on,
                    ),
                ],
            ),
        )
    return nodes


def _build_nodes(context):
    arm = context.launch_configurations.get("arm", "left")

    use_cameras = LaunchConfiguration("use_cameras")
    use_servo = LaunchConfiguration("use_servo")
    use_foxglove = LaunchConfiguration("use_foxglove")
    use_vr = LaunchConfiguration("use_vr")

    urdf_path = os.path.join(
        get_package_share_directory("flexiv_amr_description"),
        "urdf", "AICO2-Rizon4.urdf",
    )
    with open(urdf_path, "r") as f:
        robot_description = f.read()

    cameras_on = IfCondition(use_cameras)
    servo_on = IfCondition(use_servo)
    foxglove_on = IfCondition(use_foxglove)
    vr_on = IfCondition(use_vr)
    head_cam_on = IfCondition(use_cameras)

    nodes = [
        # ============================================================
        # 1. robot_state_publisher — URDF -> TF
        # ============================================================
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description,
                         "use_sim_time": False}],
        ),

        # ============================================================
        # 2. RealSense D456 cameras — color only for operator view
        # ============================================================
        _camera("cam_left", LaunchConfiguration("cam_left_serial"), cameras_on),
        _camera("cam_right", LaunchConfiguration("cam_right_serial"), cameras_on),
        _camera("cam_head", LaunchConfiguration("cam_head_serial"), head_cam_on),

        # ============================================================
        # 3. foxglove_bridge — WebSocket for Foxglove Studio in browser
        # ============================================================
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{
                "port": 8765,
                "address": "0.0.0.0",
                "send_buffer_limit": 10000000,
                "use_sim_time": False,
            }],
            condition=foxglove_on,
        ),

        # ============================================================
        # 4. move_group — planning scene (needed by Servo for collisions)
        # ============================================================
        TimerAction(
            period=MOVE_GROUP_DELAY,
            actions=[
                _include("aico2_moveit_config", "move_group.launch.py",
                         condition=servo_on),
            ],
        ),
    ]

    # ============================================================
    # 5+. Per-arm nodes: rt_bridge, servo, vr_bridge
    # ============================================================
    if arm == "both":
        shm_left = context.launch_configurations.get(
            "shm_name_left", _DEFAULT_SHM["left"])
        shm_right = context.launch_configurations.get(
            "shm_name_right", _DEFAULT_SHM["right"])
        nodes += _arm_nodes("left", shm_left, servo_on, vr_on,
                            include_vr=False)
        nodes += _arm_nodes("right", shm_right, servo_on, vr_on,
                            include_vr=False)
        nodes.append(
            TimerAction(
                period=VR_DELAY,
                actions=[
                    Node(
                        package="aico2_vr_teleop",
                        executable="vr_bridge",
                        name="vr_bridge",
                        output="screen",
                        parameters=[{
                            "hand": "both",
                            "left_planning_frame": _PLANNING_FRAMES["left"],
                            "left_arm_ns": "left_arm",
                            "left_twist_topic":
                                "/servo_node_left/delta_twist_cmds",
                            "right_planning_frame": _PLANNING_FRAMES["right"],
                            "right_arm_ns": "right_arm",
                            "right_twist_topic":
                                "/servo_node_right/delta_twist_cmds",
                        }],
                        condition=vr_on,
                    ),
                ],
            ),
        )
    else:
        shm_key = f"shm_name_{arm}"
        shm_name = context.launch_configurations.get(
            shm_key, context.launch_configurations.get(
                "shm_name", _DEFAULT_SHM.get(arm, _DEFAULT_SHM["left"])))
        nodes += _arm_nodes(arm, shm_name, servo_on, vr_on)

    # ============================================================
    # Episode recorder — button B on Quest toggles recording
    # ============================================================
    use_recorder = LaunchConfiguration("use_recorder")
    rec_arm_ns = f"{arm}_arm" if arm != "both" else "left_arm"
    nodes.append(
        Node(
            package="aico2_vr_teleop",
            executable="episode_recorder",
            name="episode_recorder",
            output="screen",
            parameters=[{
                "arm_ns": rec_arm_ns,
                "output_dir": "~/recordings",
                "record_rate": 15.0,
                "camera_topics": [
                    "/cam_left/cam_left/color/image_raw",
                    "/cam_right/cam_right/color/image_raw",
                    "/cam_head/cam_head/color/image_raw",
                ],
                "camera_names": ["cam_left", "cam_right", "cam_head"],
            }],
            condition=IfCondition(use_recorder),
        ),
    )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arm", default_value="left",
                              description="Which arm to teleop: left, right, or both"),
        DeclareLaunchArgument("use_cameras", default_value="true",
                              description="Launch the RealSense D456 cameras"),
        DeclareLaunchArgument("use_servo", default_value="true",
                              description="Launch MoveIt Servo + move_group "
                                          "(false for direct joint streaming)"),
        DeclareLaunchArgument("use_foxglove", default_value="true",
                              description="Launch foxglove_bridge for browser "
                                          "video streaming"),
        DeclareLaunchArgument("use_vr", default_value="false",
                              description="Launch VR teleop bridge (WebXR "
                                          "controller to Servo twist)"),
        DeclareLaunchArgument("use_recorder", default_value="false",
                              description="Launch episode recorder for "
                                          "pi0.5 data collection (button B "
                                          "on Quest toggles recording)"),
        DeclareLaunchArgument("cam_left_serial", default_value="_327622300610",
                              description="Serial of the left-view D456"),
        DeclareLaunchArgument("cam_right_serial", default_value="_327622300144",
                              description="Serial of the right-view D456"),
        DeclareLaunchArgument("cam_head_serial", default_value="_324422301136",
                              description="Serial of the head-mounted D456"),
        DeclareLaunchArgument("shm_name", default_value="/aico2_rt_control",
                              description="Shared memory name for rt_server "
                                          "(single-arm mode)"),
        DeclareLaunchArgument("shm_name_left", default_value="/aico2_rt_control",
                              description="Shared memory name for left arm "
                                          "rt_server (both-arm mode)"),
        DeclareLaunchArgument("shm_name_right",
                              default_value="/aico2_rt_control_right",
                              description="Shared memory name for right arm "
                                          "rt_server (both-arm mode)"),
        OpaqueFunction(function=_build_nodes),
    ])
