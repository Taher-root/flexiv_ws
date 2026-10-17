"""move_group for the two Rizon arms.

Composes with the existing bringup rather than replacing it: this launch starts
move_group only. The arm drivers, which serve the FollowJointTrajectory actions
move_group executes on, come from flexiv_amr_bringup/arms.launch.py.

robot_state_publisher is NOT started here: vr.launch.py launches RSP directly
from the generated URDF. RSP turns /joint_states into link transforms; without
it move_group has no TF and the planning scene monitor complains.

The URDF is generated at launch time from flexiv_description xacro with
post-processing renames to match the existing naming convention (Left_*, Right_*,
AGV_*).
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import xacro

_PKG = "aico2_moveit_config"

_URDF_RENAMES = [
    ('name="AICO2-4E-D1"', 'name="AICO2-Rizon4"'),
    ('"aico2_4e_d1_base_link"', '"base_link"'),
    ('"aico2_4e_d1_joint1"',    '"AGV_Joint1"'),
    ('"aico2_4e_d1_joint2"',    '"AGV_Joint2"'),
    ('"aico2_4e_d1_link1"',     '"AGV_Yaw"'),
    ('"aico2_4e_d1_link2"',     '"AGV_Pitch"'),
    ('"left_base_link"',        '"Left_link0"'),
    ('"right_base_link"',       '"Right_link0"'),
    ('"left_flange"',           '"Left_flange"'),
    ('"right_flange"',          '"Right_flange"'),
    ('"left_joint',             '"Left_joint'),
    ('"left_link',              '"Left_link'),
    ('"right_joint',            '"Right_joint'),
    ('"right_link',             '"Right_link'),
]


def _generate_urdf():
    xacro_file = os.path.join(
        get_package_share_directory('flexiv_description'),
        'urdf', 'flexiv.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={'robot_type': 'AICO2-4E-D1'})
    urdf = doc.toxml()
    for old, new in _URDF_RENAMES:
        urdf = urdf.replace(old, new)
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.urdf', prefix='aico2_', delete=False)
    tmp.write(urdf)
    tmp.close()
    return tmp.name


_URDF = _generate_urdf()


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
