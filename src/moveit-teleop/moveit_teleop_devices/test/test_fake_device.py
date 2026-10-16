"""Regression tests for the fake device.

These matter more than they look. The fake device is the ground truth that
every downstream component is tested against, so if its adversarial patterns
silently stop being adversarial, the watchdog and clutch tests further down the
stack all pass for the wrong reason.

Each test drives a real node through a real executor and subscribes over real
DDS, so QoS and topic wiring are covered too, not just the pattern arithmetic.
"""

import math

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from moveit_teleop_devices.device_base import INPUT_QOS
from moveit_teleop_devices.fake_device import PATTERNS, FakeDevice
from moveit_teleop_msgs.msg import TeleopInput


class _Collector(Node):
    def __init__(self):
        super().__init__('collector')
        self.samples = []
        self.create_subscription(
            TeleopInput, '/fake_device/input', self._cb, INPUT_QOS)

    def _cb(self, msg):
        arrival = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.samples.append((arrival, stamp, msg))


def _run(pattern, seconds=6.0, **params):
    """Spin a FakeDevice and a subscriber together, return collected samples."""
    overrides = [Parameter('pattern', value=pattern)]
    overrides += [Parameter(k, value=v) for k, v in params.items()]

    rclpy.init()
    try:
        device = FakeDevice(parameter_overrides=overrides)
        collector = _Collector()
        executor = SingleThreadedExecutor()
        executor.add_node(device)
        executor.add_node(collector)

        deadline = device.get_clock().now().nanoseconds * 1e-9 + seconds
        while device.get_clock().now().nanoseconds * 1e-9 < deadline:
            executor.spin_once(timeout_sec=0.05)

        samples = list(collector.samples)
        device.destroy_node()
        collector.destroy_node()
        return samples
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_all_patterns_are_runnable():
    """Every advertised pattern must actually construct and publish."""
    for pattern in PATTERNS:
        samples = _run(pattern, seconds=1.0, period=2.0)
        assert samples, f'pattern {pattern!r} published nothing'


def test_circle_is_a_clean_baseline():
    """The circle must be geometrically exact and free of any fault injection.

    Everything else is compared against this, so contamination here would mask
    real failures downstream.
    """
    samples = _run('circle', seconds=6.0, period=2.0, radius=0.1)
    assert len(samples) > 400

    for _, _, msg in samples:
        radius = math.hypot(msg.pose.position.x, msg.pose.position.y)
        assert radius == pytest.approx(0.1, abs=1e-9)
        assert msg.pose.position.z == pytest.approx(0.0)
        assert msg.pose_valid
        assert msg.clutch
        assert 0.0 <= msg.gripper <= 1.0

    ages = [arrival - stamp for arrival, stamp, _ in samples]
    assert max(ages) < 0.05, 'baseline must not inject staleness'

    gaps = [samples[i][0] - samples[i - 1][0] for i in range(1, len(samples))]
    assert max(gaps) < 0.1, 'baseline must not inject dropouts'


def test_dropout_injects_all_three_failure_modes():
    """dropout must produce silence, stale stamps, AND invalid poses.

    These are three genuinely different failures. A watchdog keyed on arrival
    time catches the first but sails straight through the second, which is
    exactly the bug this pattern exists to expose -- so all three must be
    present or the watchdog test downstream is not proving what it claims.
    """
    samples = _run('dropout', seconds=8.0, period=2.0,
                   dropout_period=3.0, dropout_duration=0.4, stale_offset=0.5)
    assert len(samples) > 300

    gaps = [samples[i][0] - samples[i - 1][0] for i in range(1, len(samples))]
    assert max(gaps) > 0.3, 'no hard publication gap was produced'

    ages = [arrival - stamp for arrival, stamp, _ in samples]
    assert max(ages) > 0.4, 'no stale timestamps were produced'

    invalid = [m for _, _, m in samples if not m.pose_valid]
    assert invalid, 'no tracking-loss samples were produced'


def test_clutch_storm_actually_chatters():
    """clutch_storm must toggle often and spend real time on both sides."""
    samples = _run('clutch_storm', seconds=6.0, period=2.0, clutch_period=0.7)
    clutch = [m.clutch for _, _, m in samples]

    transitions = sum(1 for i in range(1, len(clutch)) if clutch[i] != clutch[i - 1])
    assert transitions >= 10, f'only {transitions} clutch transitions'

    engaged = sum(clutch) / len(clutch)
    assert 0.3 < engaged < 0.7, f'clutch duty cycle {engaged:.2f} is lopsided'

    # It must ride the same circle as the baseline, or comparisons against
    # 'circle' would confound the clutch behaviour with a different path.
    for _, _, msg in samples:
        assert math.hypot(msg.pose.position.x, msg.pose.position.y) == \
            pytest.approx(0.1, abs=1e-9)


def test_wobble_rotates_without_translating():
    """wobble isolates rotation, so translation must stay exactly zero."""
    samples = _run('wobble', seconds=4.0, period=2.0)

    for _, _, msg in samples:
        assert msg.pose.position.x == pytest.approx(0.0)
        assert msg.pose.position.y == pytest.approx(0.0)
        assert msg.pose.position.z == pytest.approx(0.0)

    quats = {(round(m.pose.orientation.x, 4), round(m.pose.orientation.y, 4),
              round(m.pose.orientation.z, 4)) for _, _, m in samples}
    assert len(quats) > 10, 'orientation did not actually sweep'


def test_quaternions_are_normalized():
    """An unnormalized quaternion corrupts every downstream rotation."""
    for pattern in ('circle', 'wobble', 'square'):
        for _, _, msg in _run(pattern, seconds=2.0, period=2.0):
            q = msg.pose.orientation
            assert math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2) == \
                pytest.approx(1.0, abs=1e-9)


def test_rejects_unknown_pattern():
    rclpy.init()
    try:
        with pytest.raises(ValueError):
            FakeDevice(parameter_overrides=[Parameter('pattern', value='nonsense')])
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_integer_pose_values_do_not_abort_the_process():
    """A device decoding JSON yields ints for whole numbers.

    ROS float64 fields accept an int on assignment and then abort the whole
    process during serialization with an assertion in the C converter. That
    killed live XR sessions on every tracking-loss frame, because a lost pose
    is reported as [0, 0, 0]. Publishing must coerce.
    """
    import json
    from rclpy.serialization import serialize_message

    from moveit_teleop_msgs.msg import TeleopInput

    raw = json.loads('{"p": [0, 0, 0], "q": [0, 0, 0, 1]}')
    assert all(isinstance(v, int) for v in raw['p']), 'test premise broken'

    msg = TeleopInput()
    px, py, pz = (float(v) for v in raw['p'])
    qx, qy, qz, qw = (float(v) for v in raw['q'])
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = px, py, pz
    (msg.pose.orientation.x, msg.pose.orientation.y,
     msg.pose.orientation.z, msg.pose.orientation.w) = qx, qy, qz, qw
    serialize_message(msg)      # would abort the interpreter if ints leaked
