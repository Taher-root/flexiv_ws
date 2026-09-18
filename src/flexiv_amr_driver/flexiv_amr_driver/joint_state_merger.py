#!/usr/bin/env python3
"""Merge /left_arm and /right_arm joint_states into full-robot /joint_states."""

from __future__ import annotations

from typing import Dict, List, Set

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

WAIST = ["AGV_Jiont1", "AGV_Jiont2"]
LEFT = [f"Left_joint{i}" for i in range(1, 8)]
RIGHT = [f"Right_joint{i}" for i in range(1, 8)]
ALL_JOINTS: List[str] = WAIST + LEFT + RIGHT
LEFT_SET: Set[str] = set(LEFT)
RIGHT_SET: Set[str] = set(RIGHT)


class JointStateMerger(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_merger")
        self.declare_parameter("left_arm_topic", "/left_arm/joint_states")
        self.declare_parameter("right_arm_topic", "/right_arm/joint_states")
        self.declare_parameter("rate_hz", 50.0)

        left_topic = str(self.get_parameter("left_arm_topic").value)
        right_topic = str(self.get_parameter("right_arm_topic").value)
        rate = float(self.get_parameter("rate_hz").value)

        self._positions: Dict[str, float] = {n: 0.0 for n in ALL_JOINTS}
        self._velocities: Dict[str, float] = {n: 0.0 for n in ALL_JOINTS}
        self._warned_left: Set[str] = set()
        self._warned_right: Set[str] = set()

        self._pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(
            JointState,
            left_topic,
            self._left_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            right_topic,
            self._right_cb,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f"Merging {left_topic} (Left_joint*) + {right_topic} (Right_joint*) "
            f"-> /joint_states ({len(ALL_JOINTS)} joints; waist at 0)"
        )

    def _merge_cb(
        self,
        msg: JointState,
        allowed: Set[str],
        source: str,
        warned: Set[str],
    ) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name not in allowed:
                if name not in warned:
                    warned.add(name)
                    self.get_logger().error(
                        f"Ignoring joint '{name}' on {source}: wrong name for this "
                        f"topic (driver may be publishing Left_joint* from right arm)"
                    )
                continue
            self._positions[name] = float(pos)
        if msg.velocity:
            for name, vel in zip(msg.name, msg.velocity):
                if name in allowed and name in self._velocities:
                    self._velocities[name] = float(vel)

    def _left_cb(self, msg: JointState) -> None:
        self._merge_cb(msg, LEFT_SET, "left_arm_topic", self._warned_left)

    def _right_cb(self, msg: JointState) -> None:
        self._merge_cb(msg, RIGHT_SET, "right_arm_topic", self._warned_right)

    def _publish(self) -> None:
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(ALL_JOINTS)
        out.position = [self._positions[n] for n in ALL_JOINTS]
        out.velocity = [self._velocities[n] for n in ALL_JOINTS]
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = JointStateMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
