#!/usr/bin/env python3
"""Entry point for the right arm lifecycle driver."""

from aico2_left_arm_driver.arm_driver_node import spin_arm_driver


def main() -> None:
    spin_arm_driver("right_arm_driver", "right_arm")


if __name__ == "__main__":
    main()
