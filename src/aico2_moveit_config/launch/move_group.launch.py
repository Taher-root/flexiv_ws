"""move_group for the two Rizon arms.

Composes with the existing bringup rather than replacing it: this launch starts
move_group only. The arm drivers, which serve the FollowJointTrajectory actions
move_group executes on, come from flexiv_amr_bringup/arms.launch.py.

robot_state_publisher does NOT: arms.launch.py starts the drivers and the joint
state merger, nothing else. RSP is what turns /joint_states into link
transforms, so without it move_group has no TF and the planning scene monitor
complains. It comes from flexiv_amr_description/display.launch.py (or
flexiv_amr_bringup/hardware_test.launch.py, which starts a great deal more).
So the usual order is

    ros2 launch flexiv_amr_bringup arms.launch.py
    ros2 launch flexiv_amr_description display.launch.py
    ros2 launch aico2_moveit_config move_group.launch.py

Do not pass use_gui:=true to display.launch.py against a real robot: it starts
joint_state_publisher_gui, which publishes its own /joint_states and fights the
drivers for the topic.

The URDF lives in flexiv_amr_description, not here, so robot_description is
loaded by absolute path from that package's share directory. That keeps one
URDF for the whole system -- description, nav2 and MoveIt all read the same
file.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

_PKG = "aico2_moveit_config"
_URDF = os.path.join(
    get_package_share_directory("flexiv_amr_description"),
    "urdf",
    "AICO2-Rizon4.urdf",
)


def generate_launch_description():
    log_level = LaunchConfiguration("log_level")

    moveit_config = (
        MoveItConfigsBuilder("AICO2", package_name=_PKG)
        # Absolute path: MoveItConfigsBuilder joins file_path onto its own
        # package path, and pathlib keeps an absolute right-hand side as-is.
        .robot_description(file_path=_URDF)
        .robot_description_semantic(file_path="config/aico2.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="info"),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                moveit_config.to_dict(),
                {
                    # Published so RViz and anything else on the domain can get
                    # the URDF from move_group. robot_state_publisher normally
                    # publishes it too -- identical content, and publishing it
                    # here means MoveIt does not silently depend on RSP being up.
                    # TF still does: RSP is what turns /joint_states into link
                    # transforms (flexiv_amr_description/display.launch.py).
                    "publish_robot_description": True,
                    "publish_robot_description_semantic": True,
                    # RViz's MotionPlanning display needs the monitored scene.
                    # Booleans, not substitutions: a LaunchConfiguration would
                    # arrive as the string "true" and fail the param's type.
                    "publish_planning_scene": True,
                    "publish_geometry_updates": True,
                    "publish_state_updates": True,
                    "publish_transforms_updates": True,
                    "use_sim_time": False,
                },
            ],
        ),
    ])
