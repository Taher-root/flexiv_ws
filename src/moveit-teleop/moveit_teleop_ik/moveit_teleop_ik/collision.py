"""Collision handling for the IK command stream.

Two layers, deliberately different in character:

**Virtual walls** (in :mod:`ik_solver`) are constraints inside the NLP. They are
exact and lag-free, and the tool slides along them. Use them for geometry you
know about -- the table, a keep-out volume.

**State validity** (here) is a backstop for everything else: self-collision, and
whatever objects happen to be in the planning scene. It asks move_group's
``/check_state_validity`` service, which is authoritative and works on any
MoveIt robot without extra dependencies, but it is a service round-trip and so
runs at its own rate rather than per solve. Expect it to react within a frame or
two, not instantly -- which is why it is a backstop and not the primary defence.

Deliberately NOT servo's proximity deceleration. Manipulating near a table means
working deliberately close to geometry, and being slowed down there is a
nuisance, not a safety feature. The policy is chosen by the caller.
"""

from __future__ import annotations

from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

#: 'none'  - no checking at all
#: 'stop'  - refuse configurations that are actually in collision; hold the
#:           last valid command. Full speed right up to contact.
#:
#: Servo's proximity deceleration is deliberately absent: it needs a
#: distance-to-collision, which GetStateValidity does not return.
POLICIES = ('none', 'stop')


class StateValidityMonitor:
    """Asks move_group whether a configuration collides.

    Non-blocking: requests are fired asynchronously and the answer is picked up
    whenever it arrives. Waiting on the service from the solve loop would stall
    the very thing it is protecting.
    """

    def __init__(self, node, group: str, joint_names,
                 service: str = '/check_state_validity', rate: float = 20.0):
        self.node = node
        self.group = group
        self.joint_names = list(joint_names)
        self.service_name = service

        self._client = node.create_client(GetStateValidity, service)
        self._pending = None
        self._candidate = None

        self.valid = True          # optimistic until told otherwise
        self.available = False
        self.contacts = []
        self._checked = 0
        self._rejected = 0

        node.create_timer(1.0 / float(rate), self._tick)

    # ---- api --------------------------------------------------------------

    def offer(self, q) -> None:
        """Give the monitor the configuration currently being commanded."""
        self._candidate = list(float(v) for v in q)

    # ---- internals --------------------------------------------------------

    def _tick(self) -> None:
        if self._candidate is None:
            return
        if not self._client.service_is_ready():
            if self.available:
                self.node.get_logger().warn(
                    f'{self.service_name} went away; collision checking is '
                    'inactive', throttle_duration_sec=10.0)
            self.available = False
            # Fail OPEN rather than closed. A missing service should not
            # silently freeze a teleoperated arm mid-task; the warning above is
            # the honest signal, and the operator still has the clutch.
            self.valid = True
            return

        if not self.available:
            self.node.get_logger().info(
                f'collision checking active via {self.service_name}')
            self.available = True

        if self._pending is not None:
            if not self._pending.done():
                return                       # still waiting; do not pile up
            try:
                response = self._pending.result()
                self.valid = bool(response.valid)
                self.contacts = [f'{c.contact_body_1}|{c.contact_body_2}'
                                 for c in response.contacts]
                self._checked += 1
                if not self.valid:
                    self._rejected += 1
            except Exception as exc:  # noqa: BLE001
                self.node.get_logger().warn(f'validity check failed: {exc}',
                                            throttle_duration_sec=10.0)
                self.valid = True
            self._pending = None

        request = GetStateValidity.Request()
        request.group_name = self.group
        state = RobotState()
        state.joint_state = JointState()
        state.joint_state.name = self.joint_names
        state.joint_state.position = self._candidate
        state.is_diff = True
        request.robot_state = state
        self._pending = self._client.call_async(request)
