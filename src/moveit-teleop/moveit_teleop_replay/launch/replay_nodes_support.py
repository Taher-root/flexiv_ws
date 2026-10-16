"""Shared description-building for the replay launches.

Imported by both replay.launch.py and the teleop launch. Lives in its own
module because a .launch.py cannot be imported by name.
"""

from launch.substitutions import Command, LaunchConfiguration

# Slash-free on purpose. MoveIt's RobotState display derives internal names
# from its Robot Description property, and a '/' in it is an invalid node name
# -- which fails the ENTIRE RViz config load, not just that one display.
URDF_TOPIC = '/replay_robot_description'
SRDF_TOPIC = '/replay_robot_description_semantic'


MODEL_REQUIRED = (
    'episode replay needs the model of the arm the episode was recorded on: '
    'pass replay_urdf_xacro:=<path> replay_srdf_xacro:=<path> '
    '(and replay_xacro_args:= if the model takes any), or launch '
    'moveit_teleop_bringup teleop.launch.py with robot_config_package:=<pkg>, '
    "which fills all three from that robot's teleop.yaml files.preview_* keys."
)


def descriptions(context):
    """Build the URDF and SRDF commands for the recorded robot.

    Takes xacro PATHS and a free-form argument string rather than any one
    robot's options, so a second arm needs no change here. Nothing in a bag
    records which robot produced it, so a wrong model shows a plausible arm in
    the wrong pose with no error anywhere -- hence no default, rather than a
    default that is silently wrong for every arm but one.
    """
    urdf_path = LaunchConfiguration('replay_urdf_xacro').perform(context)
    srdf_path = LaunchConfiguration('replay_srdf_xacro').perform(context)
    args = LaunchConfiguration('replay_xacro_args').perform(context)
    if not urdf_path.strip() or not srdf_path.strip():
        raise RuntimeError(MODEL_REQUIRED)

    def build(path):
        # shlex-style splitting is what launch's Command does to its argument,
        # so an arg whose value contains spaces must arrive already quoted.
        return Command(['xacro ', path] + ([' ', args] if args.strip() else []))

    return build(urdf_path), build(srdf_path)
