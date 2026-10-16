"""Base class for teleoperation input devices.

A device's entire job is to answer "where is the operator's hand, are they
engaged, and what did they press" -- in the device's own frame, on the device's
own schedule. It does not know about robots, links, joints, or MoveIt.

Subclasses implement :meth:`poll` (and optionally :meth:`start`/:meth:`stop`)
and get publishing, QoS, timestamping, and lifecycle for free.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from moveit_teleop_msgs.msg import TeleopEvent, TeleopInput


#: High-rate continuous input. BEST_EFFORT with depth 1 because the newest
#: sample always supersedes the previous one -- redelivering a stale pose is
#: worse than dropping it, and a queue would add latency under load.
INPUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

#: Discrete operator actions. RELIABLE with a real queue because each of these
#: matters exactly once; losing "save episode" loses a demonstration.
EVENT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)


@dataclass
class DeviceSample:
    """One reading from a device, in the device's own frame.

    This is the plain-Python mirror of :class:`TeleopInput`. Devices build one
    of these so they never have to touch ROS message construction or clock
    handling, which keeps driver code short enough to be obviously correct.
    """

    #: (x, y, z) in the device frame.
    position: tuple = (0.0, 0.0, 0.0)
    #: (x, y, z, w) quaternion in the device frame.
    orientation: tuple = (0.0, 0.0, 0.0, 1.0)
    #: False when tracking is lost. NEVER report a stale pose as valid.
    pose_valid: bool = True
    #: Level-triggered engage. See TeleopInput.clutch for why this is a level.
    clutch: bool = False
    #: Normalized [0, 1], 0 = closed.
    gripper: float = 0.0
    gripper_valid: bool = True
    #: Discrete events observed during this poll, as TeleopEvent.type values.
    #: Each entry is (event_type, payload).
    events: list = field(default_factory=list)
    #: Device sample time in seconds. None means "use now"; devices with their
    #: own clock (a headset reporting capture time) should set it explicitly,
    #: because the watchdog measures staleness against this value.
    stamp: float = None


class TeleopDevice(Node, ABC):
    """A ROS node wrapping one input device.

    Parameters
    ----------
    rate_hz
        How often :meth:`poll` is called. Devices that block on their own
        transport should override :meth:`start` and publish from their own
        thread via :meth:`publish_sample` instead of relying on this timer.
    frame_id
        Name of the device's own reference frame. Mapping it into a ROS frame
        is the retargeter's job -- publishing raw here keeps the driver honest
        and makes coordinate bugs visible in one place instead of two.
    """

    def __init__(self, node_name: str, *, default_rate_hz: float = 100.0,
                 default_frame_id: str = 'device', **node_kwargs):
        # node_kwargs forwards to rclpy's Node, which is how tests inject
        # parameter_overrides to drive a device in-process without spawning a
        # subprocess and parsing its output.
        Node.__init__(self, node_name, **node_kwargs)

        self.declare_parameter('rate_hz', default_rate_hz)
        self.declare_parameter('frame_id', default_frame_id)

        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self._input_pub = self.create_publisher(TeleopInput, '~/input', INPUT_QOS)
        self._event_pub = self.create_publisher(TeleopEvent, '~/event', EVENT_QOS)

        self.start()
        self._timer = self.create_timer(1.0 / self.rate_hz, self._tick)

        self.get_logger().info(
            f"{node_name} publishing {self.rate_hz:.0f} Hz on "
            f"'{self._input_pub.topic_name}' (frame '{self.frame_id}')"
        )

    # ---- subclass API -----------------------------------------------------

    def start(self) -> None:
        """Open the device. Override for anything needing setup."""

    def stop(self) -> None:
        """Close the device. Override to release hardware."""

    @abstractmethod
    def poll(self) -> DeviceSample:
        """Return the current device state, or None to publish nothing.

        Called at ``rate_hz``. Returning None models a genuine gap in the
        input stream -- a dropped packet, a headset that has not produced a
        new frame -- which is distinct from returning an invalid sample. The
        watchdog must be able to tell "no data arrived" from "data arrived
        saying tracking is lost", so both paths have to be expressible.

        Must not block: this runs on the executor thread, and a device that
        stalls here starves the watchdog it is supposed to feed.
        """

    # ---- plumbing ---------------------------------------------------------

    def _tick(self) -> None:
        try:
            sample = self.poll()
        except Exception as exc:  # noqa: BLE001 - a driver fault must not kill the node
            # Deliberately degrade to "tracking lost" rather than dying. A dead
            # publisher and a device reporting invalid look identical to the
            # watchdog, but staying alive keeps events and diagnostics flowing.
            self.get_logger().error(f'poll() raised, reporting tracking loss: {exc}')
            sample = DeviceSample(pose_valid=False, clutch=False, gripper_valid=False)
        if sample is None:
            return
        self.publish_sample(sample)

    def publish_sample(self, sample: DeviceSample) -> None:
        """Publish one sample and any events it carries."""
        stamp = (
            self.get_clock().now().to_msg()
            if sample.stamp is None
            else rclpy.time.Time(nanoseconds=int(sample.stamp * 1e9)).to_msg()
        )

        msg = TeleopInput()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        # Every numeric field is coerced explicitly. ROS float64 fields accept
        # a Python int on assignment and then ABORT THE PROCESS during
        # serialization:
        #     _point_s.c:58: convert_from_py: Assertion `PyFloat_Check(field)'
        # A driver decoding JSON hits this the moment a value happens to be a
        # whole number -- [0, 0, 0] for a lost pose is the obvious case, and
        # JavaScript cannot help because JSON.stringify(0.0) emits "0".
        px, py, pz = (float(v) for v in sample.position)
        qx, qy, qz, qw = (float(v) for v in sample.orientation)
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = px, py, pz
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = qx, qy, qz, qw
        msg.pose_valid = bool(sample.pose_valid)
        msg.clutch = bool(sample.clutch)
        msg.gripper = float(sample.gripper)
        msg.gripper_valid = bool(sample.gripper_valid)
        self._input_pub.publish(msg)

        for event_type, payload in sample.events:
            self.publish_event(event_type, payload)

    def publish_event(self, event_type: int, payload: str = '') -> None:
        evt = TeleopEvent()
        evt.header.stamp = self.get_clock().now().to_msg()
        evt.header.frame_id = self.frame_id
        evt.type = int(event_type)
        evt.payload = payload
        self._event_pub.publish(evt)
        self.get_logger().info(f'event type={event_type} payload={payload!r}')

    def destroy_node(self) -> bool:
        self.stop()
        return super().destroy_node()


def spin_device(device_cls, args=None) -> None:
    """Standard main() for a device node."""
    rclpy.init(args=args)
    node = device_cls()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT/SIGTERM are how these nodes are normally stopped, including by
        # launch teardown. Treat them as a clean exit so a shutdown does not
        # look like a driver crash in the logs.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
