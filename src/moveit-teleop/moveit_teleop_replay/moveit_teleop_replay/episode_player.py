"""Replay a recorded episode into RViz.

Publishes the recorded joint states and target pose on their own topics so a
replay can run alongside a live teleop session without the two fighting over
/joint_states or over TF. See tf_prefixer for the frame side of that.

Everything a UI needs is on ``~/status``; everything it can do is one
``~/command`` service. That split is deliberate -- the RViz panel holds no
state of its own, so it cannot disagree with the player about what is loaded.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import DisplayRobotState, ObjectColor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, String

from moveit_teleop_msgs.msg import ReplayStatus
from moveit_teleop_msgs.srv import ReplayCommand

from .library import discover, load

# Topics a replay must NEVER publish on. These carry the LIVE robot's
# state; injecting recorded joint values into them corrupts what
# move_group and RViz believe the arm is doing, and on some setups drives
# it. This is not hypothetical: a launch-argument name collision pointed
# the player at /control/joint_states, and nothing downstream complained.
LIVE_TOPICS = ('/joint_states', '/control/joint_states',
               '/feedback/joint_states')

# Latched: a panel that connects after the player has settled still gets the
# episode list and the current position immediately, rather than looking empty
# until something changes.
STATUS_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

# robot_description is latched by whoever publishes it, and the player may
# well start first, so it has to be transient-local to receive it at all.
DESCRIPTION_QOS = STATUS_QOS


class EpisodePlayer(Node):

    def __init__(self):
        super().__init__('episode_player')

        self.declare_parameter('episodes_dir', '~/teleop_episodes')
        self.declare_parameter('joint_states_topic', '/replay/joint_states')
        self.declare_parameter('target_topic', '/replay/target_pose')
        # None means "the rate the episode was exported at", which is what the
        # dataset would contain.
        self.declare_parameter('rate_hz', 0.0)
        self.declare_parameter('autoload_latest', True)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('robot_description_topic',
                               '/replay_robot_description')
        # Rendered by MoveIt's RobotState display, which colours each link
        # from highlight_links. This is what makes a replay read as a
        # PREVIEW rather than as a second real arm -- the same reason
        # MoveIt draws planned paths in a flat colour instead of the
        # robot's own materials.
        self.declare_parameter('preview_color', [0.15, 0.8, 0.35, 1.0])

        self.episodes_dir = self.get_parameter('episodes_dir').value
        rate = float(self.get_parameter('rate_hz').value)
        self.rate_hz = rate if rate > 0.0 else None

        joints_topic = self.get_parameter('joint_states_topic').value
        if joints_topic in LIVE_TOPICS:
            raise RuntimeError(
                f'joint_states_topic is {joints_topic!r}, which carries the '
                'LIVE robot state. Replay must publish somewhere of its '
                f'own (default /replay/joint_states). Refusing to start.')
        self._joints = self.create_publisher(JointState, joints_topic, 10)
        self._target = self.create_publisher(
            PoseStamped, self.get_parameter('target_topic').value, 10)
        self._status = self.create_publisher(ReplayStatus, '~/status', STATUS_QOS)
        self._robot_state = self.create_publisher(
            DisplayRobotState, '~/robot_state', 10)

        # Link names come from the URDF the replay's robot_state_publisher
        # already latches, so nothing here has to know the robot.
        self._link_colors = []
        self._model_joints = set()
        self.create_subscription(
            String, self.get_parameter('robot_description_topic').value,
            self._on_description, DESCRIPTION_QOS)
        self.create_service(ReplayCommand, '~/command', self._on_command)

        self._summaries = []
        self._timeline = None
        self._index = 0
        self._playing = False
        self._speed = 1.0
        self._loop = False
        self._message = ''
        # Fractional frame position, so speeds below 1.0 do not round to a
        # standstill and speeds above it do not have to be integers.
        self._cursor = 0.0

        self._refresh()
        if self.get_parameter('autoload_latest').value and self._summaries:
            self._load(self._summaries[0].name)

        period = 1.0 / max(1e-3, float(self.get_parameter('publish_rate').value))
        self._step_period = period
        self.create_timer(period, self._tick)
        self.create_timer(0.2, self._publish_status)

        self.get_logger().info(
            f'episode player ready: {len(self._summaries)} episode(s) in '
            f'{self.episodes_dir}; publishing to '
            f"'{self.get_parameter('joint_states_topic').value}'")

    # ---- commands ---------------------------------------------------------

    def _on_command(self, request, response):
        # The command constants live on the Request class, not on the
        # service type -- and building this map is outside the try below,
        # so getting it wrong takes the node down on the first command
        # rather than returning an error.
        handlers = {
            ReplayCommand.Request.LOAD: self._cmd_load,
            ReplayCommand.Request.PLAY: self._cmd_play,
            ReplayCommand.Request.PAUSE: self._cmd_pause,
            ReplayCommand.Request.SEEK: self._cmd_seek,
            ReplayCommand.Request.STEP: self._cmd_step,
            ReplayCommand.Request.SET_SPEED: self._cmd_speed,
            ReplayCommand.Request.SET_LOOP: self._cmd_loop,
            ReplayCommand.Request.REFRESH: self._cmd_refresh,
        }
        handler = handlers.get(request.command)
        if handler is None:
            response.ok = False
            response.message = f'unknown command {request.command}'
            return response
        try:
            response.ok, response.message = handler(request)
        except Exception as exc:                      # noqa: BLE001
            # A malformed bag must not take the player down: the operator needs
            # it alive to pick a different episode.
            self.get_logger().error(f'{type(exc).__name__}: {exc}')
            response.ok, response.message = False, f'{type(exc).__name__}: {exc}'
        self._message = '' if response.ok else response.message
        self._publish_status()
        return response

    def _cmd_load(self, request):
        name = request.episode or (self._summaries[0].name
                                   if self._summaries else '')
        if not name:
            return False, 'no episodes available'
        return self._load(name)

    def _cmd_play(self, _request):
        if self._timeline is None:
            return False, 'nothing loaded'
        # Playing from the very end would look like nothing happened.
        if self._index >= self._timeline.count - 1:
            self._seek(0)
        self._playing = True
        return True, ''

    def _cmd_pause(self, _request):
        self._playing = False
        return True, ''

    def _cmd_seek(self, request):
        if self._timeline is None:
            return False, 'nothing loaded'
        self._seek(request.index)
        return True, ''

    def _cmd_step(self, request):
        if self._timeline is None:
            return False, 'nothing loaded'
        self._playing = False
        self._seek(self._index + (request.index or 1))
        return True, ''

    def _cmd_speed(self, request):
        if not 0.0 < request.speed <= 100.0:
            return False, f'speed {request.speed} out of range (0, 100]'
        self._speed = float(request.speed)
        return True, ''

    def _cmd_loop(self, request):
        self._loop = bool(request.loop)
        return True, ''

    def _cmd_refresh(self, _request):
        self._refresh()
        return True, ''

    # ---- playback ---------------------------------------------------------

    def _refresh(self):
        self._summaries = discover(self.episodes_dir)

    def _load(self, name):
        match = next((s for s in self._summaries if s.name == name), None)
        if match is None:
            # It may have been recorded since the last scan.
            self._refresh()
            match = next((s for s in self._summaries if s.name == name), None)
        if match is None:
            return False, f'no episode named {name!r}'

        timeline = load(match, rate_hz=self.rate_hz)

        # An episode recorded on a DIFFERENT robot renders nothing at all: the
        # DisplayRobotState carries that arm's joint names, the preview model
        # has none of them, and is_diff means the display simply keeps its
        # default pose. No error anywhere -- the panel says an episode is
        # loaded and the viewport stays empty.
        recorded = self._episode_joints(timeline)
        if recorded and self._model_joints and not (recorded & self._model_joints):
            return False, (
                f'{name} was recorded on a different robot: it has joints '
                f'{sorted(recorded)[:3]}... and this preview model has '
                f'{sorted(self._model_joints)[:3]}... Point '
                'replay_urdf_xacro at the arm the episode came from.')

        if timeline.count == 0:
            # Loading it anyway would leave a UI pointing at an empty episode
            # with no explanation.
            return False, (
                f'{name} resampled to zero frames: a required stream is '
                'missing or never overlapped the others')
        self._timeline = timeline
        self._playing = False
        self._seek(0)
        self.get_logger().info(
            f'loaded {name}: {timeline.count} frames, '
            f'{timeline.duration:.1f}s')
        return True, ''

    @staticmethod
    def _episode_joints(timeline) -> set:
        """Joint names the episode actually carries, from its first frame."""
        for frame in timeline.frames:
            joints = frame.values.get('joint_states')
            if joints is not None:
                return set(joints.name)
        return set()

    def _seek(self, index):
        self._index = self._timeline.clamp(index) if self._timeline else 0
        self._cursor = float(self._index)
        self._publish_frame()

    def _tick(self):
        if not self._playing or self._timeline is None:
            return
        # Advance in frames, not in samples, so `speed` means what it says
        # regardless of how fast this timer happens to run.
        frame_rate = (self._timeline.count / self._timeline.duration
                      if self._timeline.duration > 0 else 0.0)
        self._cursor += self._speed * frame_rate * self._step_period

        if self._cursor >= self._timeline.count - 1:
            if self._loop:
                self._cursor = 0.0
            else:
                self._cursor = float(self._timeline.count - 1)
                self._playing = False
        self._index = self._timeline.clamp(int(self._cursor))
        self._publish_frame()

    def _on_description(self, msg: String) -> None:
        """Cache one ObjectColor per link, so every frame is cheap to publish."""
        import xml.etree.ElementTree as ET

        try:
            root = ET.fromstring(msg.data)
        except ET.ParseError as exc:
            self.get_logger().warn(f'robot_description did not parse: {exc}')
            return
        # Kept so a loaded episode can be checked against the robot it will
        # be drawn on. Nothing in a bag records which arm produced it.
        self._model_joints = {
            j.get('name') for j in root.findall('joint')
            if j.get('type') in ('revolute', 'prismatic', 'continuous')}
        rgba = [float(v) for v in self.get_parameter('preview_color').value]
        color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
        self._link_colors = [
            ObjectColor(id=link.get('name'), color=color)
            for link in root.findall('link') if link.get('name')
        ]
        self.get_logger().info(
            f'preview colouring {len(self._link_colors)} links')

    def _publish_frame(self):
        frame = self._timeline.at(self._index) if self._timeline else None
        if frame is None:
            return
        stamp = self.get_clock().now().to_msg()

        joints = frame.values.get('joint_states')
        if joints is not None:
            # Restamp: the recorded stamps are in the past, and TF would
            # discard transforms older than its buffer.
            joints.header.stamp = stamp
            self._joints.publish(joints)

            preview = DisplayRobotState()
            preview.state.joint_state = joints
            # is_diff, so the display fills in any joint the episode did not
            # record rather than defaulting it to zero and bending the arm.
            preview.state.is_diff = True
            preview.highlight_links = self._link_colors
            self._robot_state.publish(preview)

        target = frame.values.get('ee_target')
        if target is not None:
            target.header.stamp = stamp
            self._target.publish(target)

    def _publish_status(self):
        msg = ReplayStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.episodes = [s.name for s in self._summaries]
        msg.playing = self._playing
        msg.speed = float(self._speed)
        msg.loop = self._loop
        msg.message = self._message
        if self._timeline is not None:
            frame = self._timeline.at(self._index)
            msg.episode = self._timeline.name
            msg.task = self._timeline.task
            msg.index = int(self._index)
            msg.count = int(self._timeline.count)
            msg.time = float(frame.timestamp) if frame else 0.0
            msg.duration = float(self._timeline.duration)
            msg.mode_times = [float(t) for t in self._timeline.mode_times]
            msg.mode_values = [int(v) & 0xFF for v in self._timeline.mode_values]
        self._status.publish(msg)


def main():
    rclpy.init()
    node = EpisodePlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
