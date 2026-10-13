"""Seconds-as-float <-> builtin_interfaces/Time conversion."""

from __future__ import annotations

import math

from builtin_interfaces.msg import Time


def seconds_to_ros_time(seconds: float) -> Time:
    t = Time()
    sec = math.floor(seconds)
    t.sec = int(sec)
    t.nanosec = int(round((seconds - sec) * 1e9))
    if t.nanosec >= 1_000_000_000:
        t.sec += 1
        t.nanosec -= 1_000_000_000
    return t
