"""RViz with the MoveIt Motion Planning panel.

Needs move_group.launch.py and the drivers already running: RViz reads the
planning scene move_group publishes and the TF robot_state_publisher provides.
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
    moveit_config = (
        MoveItConfigsBuilder("AICO2", package_name=_PKG)
        .robot_description(file_path=_URDF)
        .robot_description_semantic(file_path="config/aico2.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    rviz_config = os.path.join(
        get_package_share_directory(_PKG), "config", "moveit.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("rviz_config", default_value=rviz_config),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        ),
    ])
