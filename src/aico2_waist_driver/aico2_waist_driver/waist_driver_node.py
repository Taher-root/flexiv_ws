#!/usr/bin/env python3
"""Lifecycle node for the waist axes (AGV_Jiont1/2), read off one Rizon
controller's 1 kHz RDK stream independently of either arm driver.

See joint_state_architecture.md sec 3 ("Why a separate waist node"), 4.2,
and 4.4. This node never commands the robot — it only opens an RDK session
to read states() — so it can run alongside whichever arm driver(s) are
using that same controller (sec 1: "Two RDK sessions to one arm: works").
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState

from aico2_left_arm_driver.flexiv_session import FlexivSession
from aico2_left_arm_driver.offset_estimator import OffsetTracker
from aico2_left_arm_driver.ros_time import seconds_to_ros_time

_STALE_WARN_INTERVAL_SEC = 2.0
_DEFAULT_JOINTS = ["AGV_Jiont1", "AGV_Jiont2"]

# clock_offset only publishes when an estimate is refreshed (once at activate,
# then every offset_refresh_sec). Transient-local so `ros2 topic echo` and any
# other late subscriber gets the current value immediately instead of waiting
# up to 5 minutes for the next refresh.
_OFFSET_QOS = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


@dataclass(frozen=True)
class _Sample:
    """One RDK external-axis reading. Replaced whole, never mutated, so the
    poll thread and the publish timer can share it without a lock (sec 4.1
    concurrency note: whole-object attribute assignment is atomic under the
    GIL)."""

    device_timestamp: Tuple[int, int]  # RobotStates.timestamp: (sec, nanosec)
    host_mono: float
    q: List[float]
    dq: List[float]


class WaistDriverNode(LifecycleNode):
    """Publishes AGV_Jiont1 (yaw) / AGV_Jiont2 (pitch) to /joint_states."""

    def __init__(self, node_name: str = "waist_driver") -> None:
        super().__init__(node_name)
        self._declare_parameters()
        self._joint_names: List[str] = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        ) or list(_DEFAULT_JOINTS)
        self._mock = bool(self.get_parameter("mock_hardware").value)
        self._robot_sn = str(self.get_parameter("source_robot_sn").value).strip()
        self._rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._use_device_timestamp = bool(
            self.get_parameter("use_device_timestamp").value
        )
        self._offset_refresh_sec = float(self.get_parameter("offset_refresh_sec").value)
        self._offset_slew_limit = float(self.get_parameter("offset_slew_limit").value)
        self._stale_threshold_sec = float(self.get_parameter("stale_threshold_sec").value)
        self._ext_dof = 0

        self._js_pub = None
        self._offset_pub = None
        self._timer = None
        self._session: Optional[FlexivSession] = None
        self._offset: Optional[OffsetTracker] = None

        self._sample: Optional[_Sample] = None  # single-slot latest value
        self._poll_thread: Optional[threading.Thread] = None
        self._poll_stop = threading.Event()

    def _declare_parameters(self) -> None:
        self.declare_parameter("source_robot_sn", "Rizon4-063352")
        self.declare_parameter("mock_hardware", True)
        self.declare_parameter("publish_rate_hz", 200.0)
        self.declare_parameter("joint_names", list(_DEFAULT_JOINTS))
        self.declare_parameter("use_device_timestamp", True)
        self.declare_parameter("offset_refresh_sec", 300.0)
        self.declare_parameter("offset_slew_limit", 0.001)
        self.declare_parameter("stale_threshold_sec", 0.05)

    # -- lifecycle -------------------------------------------------------

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(
            f"configure: mock={self._mock} sn={self._robot_sn or '(unset)'} "
            f"joints={self._joint_names} rate={self._rate_hz} Hz"
        )
        if not self._mock:
            if not self._robot_sn:
                self.get_logger().error(
                    "mock_hardware:=false requires source_robot_sn "
                    "(Flexiv Elements serial of the controller to read from)"
                )
                return TransitionCallbackReturn.FAILURE
            try:
                self._session = FlexivSession(self._robot_sn, self.get_logger())
                self._session.connect()
                info = self._session.robot.info()
                self._ext_dof = int(info.DoF_e)
                if self._ext_dof < len(self._joint_names):
                    self.get_logger().warn(
                        f"RDK reports DoF_e={self._ext_dof} external axes, "
                        f"expected >= {len(self._joint_names)} for "
                        f"{self._joint_names}"
                    )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"flexivrdk connect failed: {exc}")
                self._session = None
                return TransitionCallbackReturn.FAILURE
            self._offset = OffsetTracker(
                states_fn=self._session.states,
                refresh_sec=self._offset_refresh_sec,
                slew_limit_sec=self._offset_slew_limit,
            )

        self._js_pub = self.create_lifecycle_publisher(
            JointState, "/joint_states", qos_profile_sensor_data
        )
        self._offset_pub = self.create_lifecycle_publisher(
            DiagnosticStatus, "~/clock_offset", _OFFSET_QOS
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        for pub in (self._js_pub, self._offset_pub):
            if pub is not None:
                pub.on_activate(state)

        if not self._mock and self._session:
            # Estimate once before the poll thread starts, while nothing
            # else is contending for states() calls.
            try:
                self._offset.refresh()
                self._publish_offset_diagnostic()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"initial offset estimate failed: {exc}")

            self._poll_stop.clear()
            self._poll_thread = threading.Thread(
                target=self._poll_loop, name="waist_poll", daemon=True
            )
            self._poll_thread.start()

        if self._timer is None:
            self._timer = self.create_timer(1.0 / self._rate_hz, self._publish_state)
        self.get_logger().info("activate: /joint_states (waist) live")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        self._poll_stop.set()
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=2.0)
            self._poll_thread = None
        for pub in (self._js_pub, self._offset_pub):
            if pub is not None:
                pub.on_deactivate(state)
        self.get_logger().info("deactivate")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        for pub in (self._js_pub, self._offset_pub):
            if pub is not None:
                self.destroy_lifecycle_publisher(pub)
        self._js_pub = None
        self._offset_pub = None
        if self._session is not None:
            self._session.disconnect()
            self._session = None
        self._offset = None
        self._sample = None
        self.get_logger().info("cleanup")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    # -- acquisition: poll thread + latest-slot (sec 4.1) -----------------

    def _poll_loop(self) -> None:
        last_ts: Optional[Tuple[int, int]] = None
        n = len(self._joint_names)
        while not self._poll_stop.is_set():
            try:
                st = self._session.states()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"states() failed: {exc}", throttle_duration_sec=1.0
                )
                time.sleep(0.05)
                continue
            # (sec, nanosec) tuple — compare exactly, don't cast to float here.
            ts = st.timestamp
            if ts != last_ts:
                last_ts = ts
                self._sample = _Sample(
                    device_timestamp=ts,
                    host_mono=time.monotonic(),
                    q=[float(x) for x in st.q[:n]],
                    dq=[float(x) for x in st.dq[:n]],
                )
            # No sleep: states() is a cached read (~4 us per sec 1); this
            # thread's job is to notice every device tick, not to pace itself.

    def _maybe_refresh_offset(self) -> None:
        if self._offset is None or not self._offset.due():
            return
        try:
            self._offset.refresh()
            self._publish_offset_diagnostic()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"offset refresh failed: {exc}", throttle_duration_sec=5.0
            )

    def _publish_offset_diagnostic(self) -> None:
        if self._offset_pub is None or self._offset is None:
            return
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = f"{self.get_name()}: clock_offset"
        status.message = f"offset={self._offset.offset_sec:.6f}s"
        status.values = [
            KeyValue(key="offset_sec", value=f"{self._offset.offset_sec:.6f}"),
            KeyValue(key="spread_sec", value=f"{self._offset.spread_sec:.6f}"),
            KeyValue(key="source_robot_sn", value=self._robot_sn),
        ]
        self._offset_pub.publish(status)

    # -- publish: timer = honest decimation (sec 4.1) ----------------------

    def _publish_state(self) -> None:
        if self._js_pub is None:
            return

        js = JointState()
        js.name = list(self._joint_names)

        if self._mock:
            js.header.stamp = self.get_clock().now().to_msg()
            js.position = [0.0] * len(self._joint_names)
            js.velocity = [0.0] * len(self._joint_names)
            self._js_pub.publish(js)
            return

        self._maybe_refresh_offset()

        sample = self._sample
        if sample is None:
            self.get_logger().warn(
                "no waist sample yet", throttle_duration_sec=_STALE_WARN_INTERVAL_SEC
            )
            return
        age = time.monotonic() - sample.host_mono
        if age > self._stale_threshold_sec:
            self.get_logger().warn(
                f"waist sample stale ({age * 1e3:.1f} ms); skipping publish",
                throttle_duration_sec=_STALE_WARN_INTERVAL_SEC,
            )
            return

        if self._use_device_timestamp and self._offset is not None:
            js.header.stamp = seconds_to_ros_time(
                self._offset.to_ros_seconds(sample.device_timestamp)
            )
        else:
            js.header.stamp = self.get_clock().now().to_msg()
        js.position = sample.q
        js.velocity = sample.dq
        self._js_pub.publish(js)


def spin_waist_driver() -> None:
    rclpy.init()
    node = WaistDriverNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    spin_waist_driver()


if __name__ == "__main__":
    main()
