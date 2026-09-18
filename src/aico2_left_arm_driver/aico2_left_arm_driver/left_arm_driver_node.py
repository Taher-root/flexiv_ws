#!/usr/bin/env python3
"""Entry point for the left arm lifecycle driver."""

from aico2_left_arm_driver.arm_driver_node import spin_arm_driver


def main() -> None:
    spin_arm_driver("left_arm_driver", "left_arm")


if __name__ == "__main__":
    main()
