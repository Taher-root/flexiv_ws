"""Flattening ROS messages into numeric features.

The LeRobot exporter that used to live here is gone. It wrote LeRobot v2.0 by
hand with no camera support at all, numbered `index` per episode instead of
across the dataset, and paired joints by their position in the message -- which
on the Piper pairs a state's joint1 with an action's gripper. Export now goes
through `convert` -> `extract` (alignment) -> `lerobot_writer` (the pinned
lerobot library), and is verified by loading the result back.
"""

from __future__ import annotations

import array

import numpy as np


def flatten_message(msg, fields=None) -> dict:
    """Flatten a ROS message into ``{name: float}``.

    Deliberately generic: it walks whatever the message actually is rather than
    switching on type, so a new sensor needs no exporter code. ``fields``
    restricts the walk to named top-level members.
    """
    out = {}

    def walk(value, prefix):
        if isinstance(value, (bool, np.bool_)):
            out[prefix] = float(value)
        elif isinstance(value, (int, float, np.integer, np.floating)):
            out[prefix] = float(value)
        elif isinstance(value, (str, bytes)):
            return          # labels, not features
        # array.array matters here: rclpy deserializes ROS float64[]/int32[]
        # fields to array.array, which is NOT a list, tuple or ndarray. Missing
        # it drops every numeric array in the message -- silently, because the
        # result is a valid-looking dataset with an empty observation vector.
        elif isinstance(value, (list, tuple, np.ndarray, array.array)):
            for i, item in enumerate(value):
                walk(item, f'{prefix}.{i}')
        elif hasattr(value, 'get_fields_and_field_types'):
            for name in value.get_fields_and_field_types():
                if name == 'header':
                    continue
                walk(getattr(value, name), f'{prefix}.{name}' if prefix else name)
        # strings and anything else are skipped: not numeric features

    if fields:
        for name in fields:
            if hasattr(msg, name):
                walk(getattr(msg, name), name)
    else:
        walk(msg, '')
    return out
