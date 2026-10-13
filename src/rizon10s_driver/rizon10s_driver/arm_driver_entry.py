#!/usr/bin/env python3
"""Entry point for the Rizon 10s arm lifecycle driver."""

from rizon10s_driver.arm_driver_node import spin_arm_driver


def main() -> None:
    spin_arm_driver("arm_driver", "arm")


if __name__ == "__main__":
    main()
