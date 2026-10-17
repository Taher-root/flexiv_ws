"""Closed-loop teleoperation: device -> retargeter -> IK -> robot.

Robot-agnostic. Nothing in this file names a link, a joint, a frame, a
controller or a vendor: the robot is selected by ONE argument, and everything
else is read either from that robot's teleop config or, at runtime, from the
model move_group already has.

    ros2 launch moveit_teleop_bringup teleop.launch.py \
        robot_config_package:=agx_arm_moveit

That reads <share>/config/teleop.yaml from the named package, and that file
names its own planning scene, RViz view and replay-preview xacros relative to
itself. `robot_config:=/path/to/teleop.yaml` also works, for a file that is not
in a package yet. Samples, including an annotated template for a new arm, are
in config/samples/.

Run it on top of the robot's own MoveIt bringup, with ITS RViz off so there is
only one window and it is the one showing the teleop target:

    ros2 launch agx_arm_moveit demo.launch.py use_rviz:=false auto_control_gate:=false
    ros2 launch moveit_teleop_bringup teleop.launch.py robot_config_package:=agx_arm_moveit

The command source is `solver`. It defaults to `ik` (moveit_teleop_ik), which
solves a full IK problem every frame with joint limits as constraints, so the
arm reconfigures instead of jamming at a bound.

`solver:=servo` selects moveit_servo instead. That path is DEPRECATED and kept
only as an A/B reference: differential IK halts a joint at its limit and cannot
redistribute motion across the arm, which is the behaviour moveit_teleop_ik was
written to replace. It receives no further tuning, and it is the only thing in
this file that still needs `xacro_mappings`.

The input device defaults to `xr`: the Quest WebXR bridge starts with the
stack, and the headset browses to https://<this-host>:8443. That needs a
certificate once per host -- see xr_device.launch.py -- because WebXR only runs
in a secure context.

    ros2 launch moveit_teleop_bringup teleop.launch.py \
        robot_config_package:=agx_arm_moveit device:=fake pattern:=circle

selects the scripted device instead, which is what tuning and hardware-free
testing use. Nothing downstream of the device changes either way.

Tuning lives in the robot's config, not in the defaults below: every argument
declared with UNSET is filled in from that file by _teleop_config.resolve, and
an argument passed on the command line overrides it.

The retargeter's target_pose is remapped straight onto the solver's pose
command topic -- both are geometry_msgs/PoseStamped, so no translating node is
needed.
"""

import sys
from pathlib import Path
from typing import List

sys.path.insert(0, str(Path(__file__).parent))

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from _teleop_config import UNSET, resolve

SERVO_POSE_TOPIC = '/servo_node/pose_target_cmds'


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device', default_value='xr', choices=['fake', 'xr', 'external'],
            description="'xr' runs the Quest WebXR bridge and is the default: "
                        'this stack exists to be driven by a headset. '
                        "'fake' runs the scripted device instead, for tuning "
                        "and for testing without hardware. 'external' starts "
                        'no device: the caller publishes on input_topic and '
                        'sets device_convention. Everything downstream is '
                        'identical -- that is the point of the device seam.'),
        DeclareLaunchArgument(
            'hand', default_value='right', choices=['left', 'right'],
            description='Which Quest controller drives the arm (device:=xr).'),
        DeclareLaunchArgument(
            'solver', default_value='ik', choices=['ik', 'servo'],
            description="'ik' = moveit_teleop_ik: full IK every frame, joint "
                        'limits as constraints, so the arm reconfigures rather '
                        'than jamming. This is the supported path. '
                        "'servo' = moveit_servo, differential IK, which halts a "
                        'joint at its bound; kept as a reference for A/B '
                        'comparison and DEPRECATED.'),
        DeclareLaunchArgument('pattern', default_value=UNSET),
        DeclareLaunchArgument('period', default_value=UNSET),
        DeclareLaunchArgument('radius', default_value=UNSET),
        DeclareLaunchArgument('plane', default_value='xy'),
        DeclareLaunchArgument('offset',
                              default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('offset_ramp', default_value='3.0'),
        DeclareLaunchArgument(
            'start_delay', default_value='0.0',
            description='Hold still this long before moving. Must outlast '
                        'homing, or the motion is absorbed into the '
                        'retargeting anchor and has no effect.'),
        DeclareLaunchArgument('move_group', default_value=UNSET),
        DeclareLaunchArgument('ready_state', default_value=UNSET),
        # Cost weights, exposed for tuning. See ik_solver.IKWeights for the
        # measurements behind the defaults.
        DeclareLaunchArgument('weight_position', default_value=UNSET),
        DeclareLaunchArgument('weight_orientation', default_value=UNSET),
        DeclareLaunchArgument('weight_regularization', default_value=UNSET),
        DeclareLaunchArgument('weight_smoothness', default_value=UNSET),
        DeclareLaunchArgument(
            'nominal_state', default_value=UNSET,
            description="Posture the IK is regularised toward. An SRDF group "
                        "state name, or 'midrange' for the joint mid-range. "
                        'Independent of ready_state, which is where the arm '
                        'starts.'),
        # Translation and rotation are scaled INDEPENDENTLY on purpose: a
        # comfortable hand-translation range is much smaller than the arm's
        # reach, so translation wants amplifying, while wrist rotation has
        # roughly the same range as the tool's and wants damping -- at 1:1 a
        # small unintended wrist roll is a large tool roll, which is what makes
        # rotation feel twitchy.
        #
        # Above 1:1 on the IK path: full IK returns the closest FEASIBLE pose,
        # so amplifying is safe -- overshooting the workspace parks the tool at
        # the boundary rather than running away. Servo has no such guarantee, so
        # it stays at half scale.
        DeclareLaunchArgument(
            'translation_scale',
            default_value=UNSET,
            description='Device metres -> robot metres. Raise for less hand '
                        'motion per unit of arm motion.'),
        DeclareLaunchArgument(
            'rotation_scale', default_value=UNSET,
            description='Device radians -> robot radians. Lower to damp '
                        'rotation; 1.0 is a direct 1:1 wrist mapping.'),
        DeclareLaunchArgument('translation_ref', default_value=UNSET),
        # 'base' = world-referenced: turn your hand in the room, the tool turns
        # the same way in the room. 'tool' composes a body-frame delta onto the
        # tool and needs a controller-to-tool alignment to feel right, which is
        # why it felt wrong.
        # Valid values are checked by _teleop_config.resolve rather than by
        # `choices`, which cannot accept the sentinel -- and which never
        # validated the config file anyway.
        DeclareLaunchArgument('rotation_ref', default_value=UNSET),
        DeclareLaunchArgument('smoothing_alpha', default_value=UNSET),
        DeclareLaunchArgument('max_translation_step', default_value=UNSET),
        DeclareLaunchArgument(
            'external_motion_threshold',
            default_value=UNSET,
            description='Unexplained EE motion (m) that forces a re-anchor. '
                        'Must exceed max_target_lead, or ordinary tracking lag '
                        'is misread as a teleport and the arm re-anchors '
                        'continuously.'),
        DeclareLaunchArgument(
            # The leash bounds how far the target may lead the arm. It exists
            # because servo lets a blocked arm fall behind while the target runs
            # away. Full IK has no such failure -- an unreachable target simply
            # yields the closest feasible pose -- so a tight leash there only
            # throttles responsiveness and makes the arm feel sluggish.
            'max_target_lead',
            default_value=UNSET,
            description='How far (m) the commanded target may lead the actual '
                        'end effector. Bounds runaway when the arm is blocked '
                        'by a joint limit. Raise for a looser, more responsive '
                        'feel; lower for a tighter, safer one.'),
        DeclareLaunchArgument('max_target_lead_angle', default_value=UNSET),
        DeclareLaunchArgument(
            'max_command_step', default_value=UNSET,
            description='Unexplained command motion (m per update) past '
                        'which the mapping is re-zeroed and the arm '
                        'holds. Catches a wall appearing or vanishing '
                        'under the target; 0 disables it.'),
        DeclareLaunchArgument('description_source', default_value='/move_group'),
        DeclareLaunchArgument('servo_node', default_value='/servo_node'),
        # Resolved from `device` so no caller has to keep the topic names and
        # the device choice in sync by hand.
        DeclareLaunchArgument(
            'input_topic',
            default_value=PythonExpression([
                "'/xr_device/input' if '", LaunchConfiguration('device'),
                "' == 'xr' else '/fake_device/input'"])),
        DeclareLaunchArgument(
            'event_topic',
            default_value=PythonExpression([
                "'/xr_device/event' if '", LaunchConfiguration('device'),
                "' == 'xr' else '/fake_device/event'"])),
        # WebXR reports x-right, y-up, -z-forward. The retargeter converts it;
        # the driver publishes raw so this lives in exactly one place.
        DeclareLaunchArgument(
            'device_convention',
            default_value=PythonExpression([
                "'webxr' if '", LaunchConfiguration('device'),
                "' == 'xr' else 'ros'"])),
        DeclareLaunchArgument('joint_states_topic',
                              default_value='/control/joint_states'),
        DeclareLaunchArgument('input_timeout', default_value=UNSET),
        DeclareLaunchArgument(
            'hardware_gate_enabled', default_value=UNSET,
            description='Drive the driver control_enable service from the mode '
                        'manager. Leave false in simulation; REQUIRED on real '
                        'hardware, where it becomes the deadman on the CAN path.'),
        DeclareLaunchArgument('hardware_gate_service', default_value=UNSET),
        DeclareLaunchArgument(
            'hardware_gate_open_for_planning', default_value=UNSET,
            description='Whether the hardware gate must also be open while '
                        'move_group executes a planned trajectory. true for a '
                        'gate that enables motion (the Piper\'s CAN gate); '
                        'false for one that selects a streaming mode the '
                        'driver\'s own trajectory action will not run '
                        'alongside (the Flexiv driver).'),
        DeclareLaunchArgument(
            'home_via_moveit', default_value=UNSET,
            # No choices= here: it cannot hold the sentinel, and _teleop_config
            # validates this against CHOICES after resolution anyway -- which
            # is also the only place that sees a value from the config file.
            description='Plan homing through move_group, so the path is '
                        'collision-checked and time-parameterised against the '
                        'robot\'s own limits. false uses the direct '
                        'interpolated move, which is also the fallback when '
                        'move_group cannot plan one.'),
        DeclareLaunchArgument(
            'home_scaling', default_value='0.3',
            description='Fraction of the model\'s velocity AND acceleration '
                        'limits used for a planned homing move.'),
        DeclareLaunchArgument(
            'goto_ready', default_value='true',
            description='Move to ready_state before enabling servo. Turning this '
                        'off on a robot parked at a singularity means servo will '
                        'emergency-stop every command.'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        # NOTHING ROBOT-SPECIFIC IS DECLARED HERE.
        #
        # arm_type / effector_type / revo2_type / tcp_offset used to live in
        # this file, which made a file named "the generic teleop bringup"
        # enumerate one vendor's arm variants and default to that vendor's
        # gripper offset. They are the ROBOT BRINGUP's arguments; this launch
        # never needed them except to build two derived strings, and both of
        # those now come from the robot config's `files:` section.
        #
        # A robot's own bringup still needs them, unchanged. See the samples in
        # config/samples/ for how a Piper expresses the same thing.
        DeclareLaunchArgument('gripper_group', default_value=UNSET),
        DeclareLaunchArgument(
            'use_gripper', default_value=UNSET,
            description='Run the gripper node. From the robot config '
                        '(gripper.enabled); a robot with no gripper group '
                        'leaves it false.'),
        DeclareLaunchArgument('arm_controller', default_value=UNSET),

        # --- the planning scene ------------------------------------------
        # Obstacles are DECLARED IN A FILE, not assembled from arguments here.
        # The previous arrangement had floor_z for the collision box and
        # tool_floor_z for the virtual wall as two independent numbers, and
        # they drifted: a wall at 0.15 over a table top at -0.005 stopped the
        # tool 155 mm in mid-air, looking like a frame misalignment rather
        # than the two settings simply disagreeing. In the config a box's wall
        # is derived from that box's bounds, so they cannot.
        DeclareLaunchArgument(
            'feedback_topic', default_value='/teleop/feedback',
            description='Robot -> operator contact feedback: the XR '
                        'bridge turns it into a controller buzz and a '
                        'tone.'),
        DeclareLaunchArgument(
            'scene_config', default_value=UNSET,
            description='Planning-scene YAML: boxes and planes the teleop '
                        'avoids. From the robot config (files.scene), '
                        'resolved relative to it. Empty means no obstacles.'),
        # Fraction of each joint's maximum velocity teleoperation may use.
        # 0.3 of MoveIt's 5.0 rad/s is 1.5 rad/s -- brisk but controllable.
        DeclareLaunchArgument('rate_limit_scale', default_value=UNSET),
        DeclareLaunchArgument(
            'solve_rate', default_value=UNSET,
            description='IK solves per second. Lower it on a slow board; the '
                        'node reports solve_time_p99 so you can see the '
                        'headroom.'),
        DeclareLaunchArgument(
            'collision_policy', default_value=UNSET,
            description="'stop' runs at full speed to contact and refuses to "
                        "enter a collision; 'none' disables checking."),
        DeclareLaunchArgument(
            'max_joint_jump', default_value=UNSET,
            description='Refuse solutions further than this (rad) from the '
                        'previous SOLUTION: an IK branch flip is valid '
                        'kinematics but unacceptable motion on real hardware. '
                        'Not compared against the measured state, which '
                        'legitimately lags.'),
        DeclareLaunchArgument(
            'jump_retry_smoothness', default_value=UNSET,
            description='On a refused jump, retry the solve once pulled this '
                        'many times harder toward the last accepted solution. '
                        '0 restores plain refusal, which leaves the arm still '
                        'until the operator moves away.'),
        DeclareLaunchArgument('record', default_value='false',
                              description='Run the episode recorder.'),
        DeclareLaunchArgument('record_dir', default_value=''),
        DeclareLaunchArgument(
            'recording_config', default_value=PathJoinSubstitution([
                FindPackageShare('moveit_teleop_bringup'),
                'config', 'recording.yaml']),
            description='Declares which topics, cameras and sensors to record.'),
        DeclareLaunchArgument('task', default_value=''),
        DeclareLaunchArgument(
            'emit_episode_events', default_value='false',
            description='Have the fake device drive a record/save cycle, for '
                        'testing the recorder unattended.'),
        # Only the DEPRECATED servo path reads this: servo builds its own copy
        # of the robot model, so it needs the same xacro arguments the robot
        # bringup used. The IK path never builds a model -- it fetches the one
        # move_group already has -- so it needs nothing here.
        DeclareLaunchArgument(
            'xacro_mappings', default_value='',
            description='xacro arguments for the DEPRECATED solver:=servo '
                        'path only, which must match the robot bringup '
                        'exactly. Unused when solver:=ik.'),
        DeclareLaunchArgument(
            'launch_servo', default_value='true',
            description='Set false if a servo_node is already running.'),
        # Review recorded episodes in THIS RViz, next to the live arm, rather
        # than in a second window. The preview robot is drawn from a
        # DisplayRobotState message, so it needs no TF of its own and cannot
        # collide with the live robot's frames.
        DeclareLaunchArgument(
            'replay', default_value='true',
            description='Add the episode replay panel and preview robot.'),
        # The preview robot's model, from the robot config (files.preview_urdf
        # / files.preview_srdf / files.preview_xacro_args). Leave them out of
        # the config and the replay preview robot simply does not start -- the
        # panel and the recorded episodes still work.
        DeclareLaunchArgument('replay_urdf_xacro', default_value=UNSET),
        DeclareLaunchArgument('replay_srdf_xacro', default_value=UNSET),
        DeclareLaunchArgument('replay_xacro_args', default_value=UNSET),
        # RViz view. A robot wants its own: the TF whitelist names this arm's
        # tip link, and showing every frame instead makes the gap between
        # commanded and actual impossible to see, which is the point of it.
        # From the robot config (files.rviz); falls back to the stack's own.
        DeclareLaunchArgument('rviz_config', default_value=UNSET),
        DeclareLaunchArgument(
            'episodes_dir', default_value=UNSET,
            description='Where the replay panel looks for episodes. Unset '
                        'means "wherever the recorder writes", taken from '
                        'recording_config, so the two cannot disagree.'),
        DeclareLaunchArgument(
            'autoload_latest', default_value='false',
            description='Load the newest episode at startup. Off here, so a '
                        'live session does not open with a ghost arm on top '
                        'of the real one; the panel still lists everything.'),

        # --- the robot ------------------------------------------------------
        # This is the entire robot-specific surface of this launch file. Every
        # argument above whose default reads UNSET is filled in from the config
        # it names, including the paths to the planning scene, the RViz view
        # and the preview robot's xacros. It is a real source of truth, not
        # documentation: change a value there and it changes what runs.
        DeclareLaunchArgument(
            'robot_config_package', default_value='',
            description='The arm\'s MoveIt config package. Reads '
                        '<share>/config/teleop.yaml from it. This is the '
                        'normal way to select a robot: '
                        'robot_config_package:=agx_arm_moveit'),
        DeclareLaunchArgument(
            'robot_config', default_value=UNSET,
            description='Explicit path to a teleop config, for a file not in '
                        'a package yet. Wins over robot_config_package. '
                        'Samples: moveit_teleop_bringup/config/samples/.'),

        # Fills in the UNSET configurations. Must run AFTER every
        # DeclareLaunchArgument (so their sentinels are in the context) and
        # BEFORE every Node (so they see resolved values) -- which is exactly
        # here, and is why this action's position in the list is load-bearing.
        OpaqueFunction(function=resolve),

        # 1. Servo itself. Deprecated: kept selectable so the two command
        #    sources can still be compared, but 'ik' is the supported path and
        #    the default. See the note in the module docstring.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('moveit_teleop_bringup'),
                'launch', 'servo.launch.py'])),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('launch_servo'), "' == 'true' and '",
                LaunchConfiguration('solver'), "' == 'servo'"])),
            launch_arguments={
                'move_group_name': LaunchConfiguration('move_group'),
                'xacro_mappings': LaunchConfiguration('xacro_mappings'),
            }.items()),

        # The full-IK alternative. Same input topic, same controller.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('moveit_teleop_bringup'),
                'launch', 'ik.launch.py'])),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('solver'), "' == 'ik'"])),
            launch_arguments={
                'move_group': LaunchConfiguration('move_group'),
                'description_source': LaunchConfiguration('description_source'),
                'joint_states_topic': LaunchConfiguration('joint_states_topic'),
                # Separate from ready_state on purpose: the arm still STARTS
                # at ready_state, this only chooses what the solver is
                # regularised toward. 'midrange' uses the joint mid-range.
                'nominal_state': LaunchConfiguration('nominal_state'),
                'weight_position': LaunchConfiguration('weight_position'),
                'weight_orientation': LaunchConfiguration('weight_orientation'),
                'weight_regularization':
                    LaunchConfiguration('weight_regularization'),
                'weight_smoothness': LaunchConfiguration('weight_smoothness'),
                'solve_rate': LaunchConfiguration('solve_rate'),
                'rate_limit_scale': LaunchConfiguration('rate_limit_scale'),
                'collision_policy': LaunchConfiguration('collision_policy'),
                'max_joint_jump': LaunchConfiguration('max_joint_jump'),
                'jump_retry_smoothness':
                    LaunchConfiguration('jump_retry_smoothness'),
                # The solver reads the SAME file the collision objects come
                # from, and derives each wall from its box's bounds.
                'scene_config': LaunchConfiguration('scene_config'),
            }.items()),

        # Episode replay: the player plus the two descriptions the preview
        # robot is built from. No RViz of its own -- the panel and the preview
        # display live in teleop.rviz below.
        #
        # GroupAction, and every argument passed EXPLICITLY, because
        # IncludeLaunchDescription shares this file's launch-configuration
        # scope. A same-named argument crosses over silently in both
        # directions, and both directions bit here: the replay's rate_hz (0.0,
        # meaning "use the episode's own rate") reached the fake device and
        # divided by zero, and this file's joint_states_topic
        # (/control/joint_states) reached the player and aimed recorded
        # playback at the LIVE robot's state topic. The replay_ prefix and the
        # group between them make that impossible rather than unlikely.
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('moveit_teleop_replay'),
                    'launch', 'replay_nodes.launch.py'])),
                launch_arguments={
                    'replay_episodes_dir': LaunchConfiguration('episodes_dir'),
                    'replay_urdf_xacro': LaunchConfiguration('replay_urdf_xacro'),
                    'replay_srdf_xacro': LaunchConfiguration('replay_srdf_xacro'),
                    'replay_xacro_args': LaunchConfiguration('replay_xacro_args'),
                    'replay_autoload_latest':
                        LaunchConfiguration('autoload_latest'),
                    'replay_joint_states_topic': '/replay/joint_states',
                    'replay_target_topic': '/replay/target_pose',
                    'replay_rate_hz': '0.0',
                    'replay_preview_color': '[0.15, 0.8, 0.35, 1.0]',
                }.items()),
        ], condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('replay'), "' == 'true' and '",
            LaunchConfiguration('replay_urdf_xacro'), "' != ''"]))),

        # The declared obstacles as real planning-scene geometry, so the
        # collision checker and MoveIt's planner both see them. The IK
        # node reads the same file for its virtual walls.
        Node(
            package='moveit_teleop_core',
            executable='scene_objects',
            name='scene_objects',
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('scene_config'),
                "' != ''"])),
            parameters=[{
                'description_source': LaunchConfiguration('description_source'),
                'scene_config': LaunchConfiguration('scene_config'),
            }],
        ),

        # 2. Mode manager: reaches a servoable posture, enables POSE mode, then
        #    owns the watchdog and arbitration for the rest of the session.
        Node(
            package='moveit_teleop_core',
            executable='mode_manager',
            name='mode_manager',
            output='screen',
            parameters=[{
                'move_group': LaunchConfiguration('move_group'),
                'ready_state': LaunchConfiguration('ready_state'),
                'description_source': LaunchConfiguration('description_source'),
                'servo_node': PythonExpression([
                    "'ik_node' if '", LaunchConfiguration('solver'),
                    "' == 'ik' else '", LaunchConfiguration('servo_node'), "'"]),
                'command_source': LaunchConfiguration('solver'),
                'joint_states_topic': LaunchConfiguration('joint_states_topic'),
                'goto_ready': LaunchConfiguration('goto_ready'),
                'input_topic': LaunchConfiguration('input_topic'),
                'event_topic': LaunchConfiguration('event_topic'),
                'input_timeout': LaunchConfiguration('input_timeout'),
                'hardware_gate_enabled':
                    LaunchConfiguration('hardware_gate_enabled'),
                'hardware_gate_service':
                    LaunchConfiguration('hardware_gate_service'),
                'hardware_gate_open_for_planning':
                    LaunchConfiguration('hardware_gate_open_for_planning'),
                # Homing is PLANNED through move_group so its path is
                # collision-checked; it falls back to a direct interpolated
                # move if move_group cannot plan one. See doc/architecture.md.
                'home_via_moveit': LaunchConfiguration('home_via_moveit'),
                'home_scaling': LaunchConfiguration('home_scaling'),
                # Only wait for a scene that is actually being applied.
                # Planning before scene_objects has published the table is
                # collision-aware against an empty world, which looks safe and
                # is not; with no scene configured there is nothing to wait for.
                'scene_ready_topic': PythonExpression([
                    "'scene_objects/applied' if '",
                    LaunchConfiguration('scene_config'), "' else ''"]),
            }],
        ),

        # 2b. Nothing else enforces joint position limits: mock hardware
        #     echoes commands, JTC does not clamp, and servo halts a joint at
        #     its bound but cannot rescue one pushed beyond it.
        Node(
            package='moveit_teleop_core',
            executable='joint_limit_guard',
            name='joint_limit_guard',
            output='screen',
            parameters=[{
                'description_source': LaunchConfiguration('description_source'),
                'input_topic': 'servo/joint_trajectory_raw',
                'output_topic': PythonExpression([
                    "'/", LaunchConfiguration('arm_controller'),
                    "' + '/joint_trajectory'"]),
            }],
        ),

        # 3. The input device: scripted, or the Quest.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('moveit_teleop_bringup'),
                'launch', 'fake_device.launch.py'])),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('device'), "' == 'fake'"])),
            launch_arguments={
                'pattern': LaunchConfiguration('pattern'),
                'period': LaunchConfiguration('period'),
                'radius': LaunchConfiguration('radius'),
                'plane': LaunchConfiguration('plane'),
                'offset': LaunchConfiguration('offset'),
                'offset_ramp': LaunchConfiguration('offset_ramp'),
                'start_delay': LaunchConfiguration('start_delay'),
                'emit_episode_events': LaunchConfiguration('emit_episode_events'),
            }.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('moveit_teleop_bringup'),
                'launch', 'xr_device.launch.py'])),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('device'), "' == 'xr'"])),
            launch_arguments={
                'hand': LaunchConfiguration('hand'),
                'feedback_topic': LaunchConfiguration('feedback_topic'),
            }.items()),

        # 4. Retargeting, with its output wired to servo by remapping.
        Node(
            package='moveit_teleop_core',
            executable='retarget_node',
            name='retarget_node',
            output='screen',
            remappings=[('~/target_pose', PythonExpression([
                "'ik_node/pose_target_cmds' if '", LaunchConfiguration('solver'),
                "' == 'ik' else '", SERVO_POSE_TOPIC, "'"]))],
            parameters=[{
                'move_group': LaunchConfiguration('move_group'),
                'description_source': LaunchConfiguration('description_source'),
                'input_topic': LaunchConfiguration('input_topic'),
                'status_topic': 'mode_manager/status',
                'translation_scale': LaunchConfiguration('translation_scale'),
                'rotation_scale': LaunchConfiguration('rotation_scale'),
                'translation_ref': LaunchConfiguration('translation_ref'),
                'rotation_ref': LaunchConfiguration('rotation_ref'),
                'device_convention': LaunchConfiguration('device_convention'),
                'smoothing_alpha': LaunchConfiguration('smoothing_alpha'),
                'max_translation_step': LaunchConfiguration('max_translation_step'),
                'external_motion_threshold':
                    LaunchConfiguration('external_motion_threshold'),
                'max_target_lead': LaunchConfiguration('max_target_lead'),
                'max_target_lead_angle':
                    LaunchConfiguration('max_target_lead_angle'),
                'max_command_step':
                    LaunchConfiguration('max_command_step'),
                # The retargeter projects the target onto these walls BEFORE
                # leashing it, which is what makes the tool slide along a
                # surface instead of pinning against it and then snapping
                # across the workspace when it comes free.
                'scene_config': LaunchConfiguration('scene_config'),
                'feedback_topic': LaunchConfiguration('feedback_topic'),
            }],
        ),

        Node(
            package='moveit_teleop_core',
            executable='gripper_node',
            name='gripper_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_gripper')),
            parameters=[{
                'gripper_group': LaunchConfiguration('gripper_group'),
                'description_source': LaunchConfiguration('description_source'),
                'input_topic': LaunchConfiguration('input_topic'),
                'status_topic': 'mode_manager/status',
                'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            }],
        ),

        Node(
            package='moveit_teleop_record',
            executable='episode_recorder',
            name='episode_recorder',
            output='screen',
            condition=IfCondition(LaunchConfiguration('record')),
            parameters=[{
                'config_file': LaunchConfiguration('recording_config'),
                'output_dir': LaunchConfiguration('record_dir'),
                'task': LaunchConfiguration('task'),
                'event_topic': LaunchConfiguration('event_topic'),
                'status_topic': 'mode_manager/status',
                # Which topics carry the action, the device input and the
                # solver status depends on HOW THIS STACK WAS LAUNCHED, not on
                # the robot, so _teleop_config computes them and recording.yaml
                # refers to them as ${...}. Without this the config is correct
                # for servo+fake and silently unusable everywhere else.
                'topic_substitutions': ParameterValue(
                    LaunchConfiguration('topic_substitutions'),
                    value_type=List[str]),
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='teleop_rviz',
            output='log',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
    ])
