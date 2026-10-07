"""VR teleop session: cameras + video streaming + MoveIt Servo + RT bridge.

One command brings up everything the operator needs for VR teleop with live
camera feeds, collision-checked arm motion, and the 1 kHz RT control loop:

  ros2 launch flexiv_amr_bringup teleop.launch.py use_vr:=true

What runs (in staged order):

  1. robot_state_publisher            URDF -> TF
  2. rt_bridge (ns /<arm>_arm)        shm <-> ROS (joint_states, servo, FJT)
  3. RealSense D456 cameras            /cam_left, /cam_right (+ /cam_head if no VR)
  4. foxglove_bridge                   WebSocket -> Foxglove Studio in browser
  5. move_group                        planning scene for collision checking
  6. servo_node                        TwistStamped -> JointTrajectory at 100 Hz
  7. vr_bridge (opt)                   WebXR poses -> Servo twist at port 8181

rt_server must already be running in a separate (ROS-free) terminal.

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


def _build_nodes(context):
    arm = context.launch_configurations.get("arm", "left")
    arm_ns = f"{arm}_arm"
    planning_frame = _PLANNING_FRAMES.get(arm, "Left_link0")

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
    head_cam_on = IfCondition(PythonExpression([
        "'", use_cameras, "'.lower() == 'true' and '",
        use_vr, "'.lower() != 'true'",
    ]))

    return [
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
        # 2. rt_bridge — shm <-> ROS, namespaced to /<arm>_arm
        # ============================================================
        Node(
            package="aico2_rt_control",
            executable="rt_bridge",
            name="rt_bridge",
            namespace=arm_ns,
            output="screen",
            parameters=[{"shm_name": LaunchConfiguration("shm_name")}],
        ),

        # ============================================================
        # 3. RealSense D456 cameras — color only for operator view
        # ============================================================
        _camera("cam_left", LaunchConfiguration("cam_left_serial"), cameras_on),
        _camera("cam_right", LaunchConfiguration("cam_right_serial"), cameras_on),
        _camera("cam_head", LaunchConfiguration("cam_head_serial"), head_cam_on),

        # ============================================================
        # 4. foxglove_bridge — WebSocket for Foxglove Studio in browser
        #    Open https://app.foxglove.dev, connect to ws://<ip>:8765
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
        # 5. move_group — planning scene (needed by Servo for collisions)
        # ============================================================
        TimerAction(
            period=MOVE_GROUP_DELAY,
            actions=[
                _include("aico2_moveit_config", "move_group.launch.py",
                         condition=servo_on),
            ],
        ),

        # ============================================================
        # 6. MoveIt Servo — TwistStamped -> collision-checked JointTrajectory
        # ============================================================
        TimerAction(
            period=SERVO_DELAY,
            actions=[
                _include("aico2_moveit_config", "servo.launch.py",
                         launch_arguments={"arm": arm},
                         condition=servo_on),
            ],
        ),

        # ============================================================
        # 7. VR bridge — WebXR controller poses -> Servo twist commands
        #    Starts after Servo so the twist topic is ready.
        #    Connect Quest 3 via ADB: adb reverse tcp:8181 tcp:8181
        # ============================================================
        TimerAction(
            period=VR_DELAY,
            actions=[
                Node(
                    package="aico2_vr_teleop",
                    executable="vr_bridge",
                    name="vr_bridge",
                    output="screen",
                    parameters=[{"planning_frame": planning_frame}],
                    condition=vr_on,
                ),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arm", default_value="left",
                              description="Which arm to teleop: left or right"),
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
        DeclareLaunchArgument("cam_left_serial", default_value="_327622300610",
                              description="Serial of the left-view D456"),
        DeclareLaunchArgument("cam_right_serial", default_value="_327622300144",
                              description="Serial of the right-view D456"),
        DeclareLaunchArgument("cam_head_serial", default_value="_324422301136",
                              description="Serial of the head-mounted D456"),
        DeclareLaunchArgument("shm_name", default_value="/aico2_rt_control",
                              description="Shared memory name for rt_server"),
        OpaqueFunction(function=_build_nodes),
    ])
