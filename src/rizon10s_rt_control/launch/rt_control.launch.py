"""Launch rt_server and rt_bridge for a single Rizon 10s arm.

rt_server is a standalone binary (not a ROS node) — it takes the robot serial
as a CLI argument and communicates with rt_bridge over POSIX shared memory.

rt_bridge is the ROS node that serves FollowJointTrajectory and publishes
/joint_states, bridging MoveIt to the 1 kHz RT loop.

Usage:
    # Mock (no hardware, just rt_bridge):
    ros2 launch rizon10s_rt_control rt_control.launch.py

    # Real hardware:
    ros2 launch rizon10s_rt_control rt_control.launch.py \
        robot_sn:=Rizon10s-XXXXXX mock:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node


def generate_launch_description():
    robot_sn = LaunchConfiguration("robot_sn")
    mock = LaunchConfiguration("mock")

    arm_joints = [f"joint{i}" for i in range(1, 8)]

    return LaunchDescription([
        DeclareLaunchArgument("robot_sn", default_value="Rizon10s-XXXXXX",
                              description="Flexiv robot serial number"),
        DeclareLaunchArgument("mock", default_value="true",
                              description="Skip rt_server (mock mode)"),

        ExecuteProcess(
            cmd=[FindExecutable(name="rt_server"), robot_sn],
            name="rt_server",
            output="screen",
            condition=UnlessCondition(mock),
        ),

        Node(
            package="aico2_rt_control",
            executable="rt_bridge",
            name="rt_bridge",
            namespace="arm",
            output="screen",
            parameters=[{
                "joint_names": arm_joints,
                "servo_joint_names": arm_joints,
            }],
        ),
    ])
