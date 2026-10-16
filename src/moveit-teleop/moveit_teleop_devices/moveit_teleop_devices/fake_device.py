"""A scripted teleoperation device.

This is the v1 workhorse. Everything downstream -- retargeting, the mode
manager, servo, recording -- is developed and regression-tested against this
node, with no headset and no robot hardware involved.

Two of the patterns exist specifically to break things:

``clutch_storm``
    Engages and disengages rapidly while moving fast. Any anchoring bug in the
    retargeter shows up here as a pose discontinuity.
``dropout``
    Stops publishing, and publishes deliberately stale timestamps. Any missing
    watchdog path shows up here as an arm that keeps moving after the operator
    is gone.

Poses are emitted in the device's own frame with an intentionally non-trivial
orientation offset, so that a retargeter which quietly ignores rotation cannot
pass by accident.
"""

import math

from moveit_teleop_msgs.msg import TeleopEvent

from .device_base import DeviceSample, TeleopDevice, spin_device

PATTERNS = ('circle', 'square', 'ramp', 'wobble', 'clutch_storm', 'dropout', 'static')


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple:
    """Return (x, y, z, w) for an intrinsic Z-Y-X rotation."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class FakeDevice(TeleopDevice):
    """Publishes a deterministic, repeatable input stream."""

    def __init__(self, **node_kwargs):
        super().__init__('fake_device', default_rate_hz=100.0,
                         default_frame_id='fake_device', **node_kwargs)

        self.declare_parameter('pattern', 'circle')
        self.declare_parameter('radius', 0.10)          # m
        # Which plane the circle (and square) is swept in.
        #
        # 'xy' is horizontal at a fixed height, which is the right default
        # for exercising reach and rotation -- but it can never touch a
        # table, so it cannot test a virtual wall or a floor collision
        # object at all. 'xz' sweeps vertically and drives straight
        # through one.
        self.declare_parameter('plane', 'xy')
        # Where the pattern is centred, as a device-frame offset from the
        # pose at engage.
        #
        # Retargeting is RELATIVE, so a constant offset would cancel out
        # entirely -- the circle would still be centred on wherever the
        # tool was when the clutch closed. It is therefore ramped in from
        # zero, which both makes it take effect and keeps the per-update
        # step well under max_translation_step. Without the ramp the first
        # sample is a teleport and the retargeter rejects every update.
        self.declare_parameter('offset', [0.0, 0.0, 0.0])
        self.declare_parameter('offset_ramp', 3.0)     # s to reach it
        # Hold the start pose for this long before moving at all.
        #
        # Retargeting anchors when the clutch closes, and everything the
        # device did BEFORE that is absorbed into the anchor and cancels.
        # The stack spends its first few seconds homing, so a pattern that
        # starts immediately has already finished its ramp by the time
        # anyone is listening -- which is why an offset appeared to do
        # nothing at all. Waiting past engage makes the motion real.
        self.declare_parameter('start_delay', 0.0)     # s
        self.declare_parameter('period', 6.0)           # s per cycle
        self.declare_parameter('clutch_period', 0.7)    # s, clutch_storm only
        self.declare_parameter('dropout_period', 3.0)   # s between dropouts
        self.declare_parameter('dropout_duration', 0.4)  # s of silence
        self.declare_parameter('stale_offset', 0.5)     # s of fake staleness
        self.declare_parameter('emit_episode_events', False)

        self.plane = str(self.get_parameter('plane').value).lower()
        self.offset = [float(v) for v in self.get_parameter('offset').value]
        if len(self.offset) != 3:
            raise ValueError(f'offset must be [x, y, z], got {self.offset}')
        self.offset_ramp = max(1e-3, float(
            self.get_parameter('offset_ramp').value))
        self.start_delay = max(0.0, float(
            self.get_parameter('start_delay').value))
        if self.plane not in ('xy', 'xz', 'yz'):
            raise ValueError(
                f"plane must be 'xy', 'xz' or 'yz', got {self.plane!r}")
        self.pattern = str(self.get_parameter('pattern').value)
        if self.pattern not in PATTERNS:
            raise ValueError(
                f"unknown pattern {self.pattern!r}; expected one of {PATTERNS}")

        self.radius = float(self.get_parameter('radius').value)
        self.period = float(self.get_parameter('period').value)
        self.clutch_period = float(self.get_parameter('clutch_period').value)
        self.dropout_period = float(self.get_parameter('dropout_period').value)
        self.dropout_duration = float(self.get_parameter('dropout_duration').value)
        self.stale_offset = float(self.get_parameter('stale_offset').value)
        self.emit_episode_events = bool(
            self.get_parameter('emit_episode_events').value)

        self._t0 = self._now()
        self._episode_state = 0  # 0=not started, 1=recording, 2=saved

        self.get_logger().info(
            f"pattern='{self.pattern}' radius={self.radius} period={self.period}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def poll(self):
        t = self._now() - self._t0 - self.start_delay
        if t < 0.0:
            # Hold the start pose so the retargeter can anchor on it.
            t = 0.0
        # Phase in [0, 1) through one cycle of the pattern.
        phase = (t % self.period) / self.period

        pos, rpy = self._trajectory(t, phase)

        # Slide the whole pattern to its working centre. Smoothstep rather
        # than linear so it starts and ends at zero velocity: a linear ramp
        # steps at full speed on its first sample, which the retargeter reads
        # as a teleport.
        if any(self.offset):
            u = min(1.0, t / self.offset_ramp)
            ramp = u * u * (3.0 - 2.0 * u)
            pos = tuple(p + ramp * o for p, o in zip(pos, self.offset))

        # Clutch defaults to engaged so the simple patterns drive the arm
        # immediately; only clutch_storm deliberately chatters it.
        clutch = True
        pose_valid = True
        stamp = None

        if self.pattern == 'clutch_storm':
            clutch = (t % self.clutch_period) < (self.clutch_period * 0.5)
        elif self.pattern == 'dropout':
            into = t % self.dropout_period
            if into < self.dropout_duration:
                # Hard gap: publish nothing at all, as a lost packet would.
                return None
            if into < self.dropout_duration * 2.0:
                # Subtler and more dangerous: keep publishing, but with an old
                # timestamp. A watchdog keyed on arrival time rather than
                # header.stamp will sail straight through this.
                stamp = self._now() - self.stale_offset
            elif into < self.dropout_duration * 3.0:
                pose_valid = False

        # Gripper cycles out of phase with position so a mix-up between the two
        # is immediately visible rather than coincidentally correct.
        gripper = 0.5 * (1.0 - math.cos(2.0 * math.pi * phase + 1.0))

        events = self._episode_events(t)

        return DeviceSample(
            position=pos,
            orientation=_quat_from_rpy(*rpy),
            pose_valid=pose_valid,
            clutch=clutch,
            gripper=gripper,
            gripper_valid=True,
            events=events,
            stamp=stamp,
        )

    def _trajectory(self, t: float, phase: float):
        """Return (position, rpy) for the configured pattern."""
        r = self.radius
        theta = 2.0 * math.pi * phase

        if self.pattern == 'static':
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

        if self.pattern == 'ramp':
            # Triangle wave along x: tests monotonic tracking and the reversal.
            x = r * (4.0 * abs(phase - 0.5) - 1.0)
            return (x, 0.0, 0.0), (0.0, 0.0, 0.0)

        if self.pattern == 'square':
            # Perimeter walk; the corners are step changes in velocity, which
            # is what makes this a better smoothing test than the circle.
            side = int(phase * 4.0) % 4
            u = (phase * 4.0) % 1.0
            x, y = {
                0: (-r + 2 * r * u, -r),
                1: (r, -r + 2 * r * u),
                2: (r - 2 * r * u, r),
                3: (-r, r - 2 * r * u),
            }[side]
            return self._in_plane(x, y), (0.0, 0.0, 0.0)

        if self.pattern == 'wobble':
            # Position held still, orientation swept: isolates rotation
            # retargeting from translation retargeting.
            return (0.0, 0.0, 0.0), (0.4 * math.sin(theta),
                                     0.4 * math.sin(theta * 0.5),
                                     0.4 * math.cos(theta))

        # circle, clutch_storm and dropout all ride the same circle so their
        # results are directly comparable against the plain 'circle' baseline.
        return self._in_plane(r * math.cos(theta), r * math.sin(theta)), \
            (0.0, 0.0, 0.0)

    def _in_plane(self, a: float, b: float) -> tuple:
        """Place a 2-D pattern coordinate into the configured plane."""
        if self.plane == 'xz':
            return (a, 0.0, b)
        if self.plane == 'yz':
            return (0.0, a, b)
        return (a, b, 0.0)

    def _episode_events(self, t: float):
        """Drive one record/save cycle, for testing the recorder unattended."""
        if not self.emit_episode_events:
            return []
        if self._episode_state == 0 and t > 2.0:
            self._episode_state = 1
            return [(TeleopEvent.RECORD_START, 'fake_episode')]
        if self._episode_state == 1 and t > 2.0 + 2.0 * self.period:
            self._episode_state = 2
            return [(TeleopEvent.RECORD_STOP, ''), (TeleopEvent.EPISODE_SAVE, '')]
        return []


def main(args=None):
    spin_device(FakeDevice, args=args)


if __name__ == '__main__':
    main()
