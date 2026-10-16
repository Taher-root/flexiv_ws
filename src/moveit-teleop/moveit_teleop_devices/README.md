# moveit_teleop_devices

Teleoperation input devices. Each one publishes the same two topics and knows
nothing about robots, kinematics or solvers:

```
~/input  TeleopInput   pose + clutch + grip, at device rate   BEST_EFFORT depth 1
~/event  TeleopEvent   discrete presses: save, discard, home     RELIABLE   depth 10
```

Everything downstream reads only those, so adding a device — a different
headset, a 6-DOF mouse, a leader arm — means adding one node here and changing
nothing else. Poses are published in the device's **own** convention; the
retargeter converts (`device_convention`), so the convention lives in exactly
one place.

---

## `xr_device` — Meta Quest over WebXR

Serves a WebXR page over HTTPS; the headset's browser opens it and streams
controller poses back over the same socket. No app to install, no Link cable.

| | |
|---|---|
| Publishes | `~/input` (`TeleopInput`, ~72 Hz — the headset's refresh), `~/event` (`TeleopEvent`) |
| Subscribes | `feedback_topic` (`TeleopFeedback`) → haptic pulses on wall contact |
| Serves | `https://<host>:8443`, from a certificate you generate once per host |

| Parameter | Default | |
|---|---|---|
| `hand` | `right` | which controller drives the arm |
| `host` / `port` | `0.0.0.0` / `8443` | WebXR **requires** HTTPS, so there is no plain-HTTP option |
| `certfile` / `keyfile` | `''` | **Required.** WebXR only runs in a secure context, so the page must be HTTPS even on a LAN. The node refuses to start without them and names the command; generate a pair once per host with `make_cert.sh <lan-ip>` |
| `web_dir` | `''` | `''` serves the bundled page (`web/`) |
| `clutch_threshold` | `0.5` | trigger value that counts as clutched |
| `gripper_deadzone` | `0.03` | grip noise suppressed around 0 and 1 |
| `feedback_topic` | `/teleop/feedback` | |
| `haptics_enabled` | `true` | |

Two startup checks exist because both failures look like "the headset will not
connect": the node prints every reachable address the headset should try, and
warns about a firewall **only when `ufw` is actually filtering** — `systemctl
is-active ufw` reports active even after `ufw disable`, so the authoritative
flag is read instead.

A stale instance holding port 8443 reports as `address already in use`; the
error message says so and names `pkill -f xr_device`.

## `fake_device` — scripted motion, no hardware

How everything downstream is tested: the retargeter, the solver, the recorder
and the whole launch graph behave identically whether the input came from a
headset or from here.

| Parameter | Default | |
|---|---|---|
| `pattern` | `circle` | `circle`, `square`, `ramp`, `wobble`, `clutch_storm`, `dropout`, `static` |
| `radius` / `period` / `plane` | `0.1` / `6.0` / `xy` | its geometry and speed |
| `offset` / `offset_ramp` / `start_delay` | `[0,0,0]` / `3.0` / `0.0` | where it starts and how it gets there |
| `clutch_period` | `0.7` | fraction of each cycle spent clutched — exercises re-anchoring |
| `dropout_period` / `dropout_duration` | `3.0` / `0.4` | synthetic tracking loss |
| `stale_offset` | `0.5` | publish with a stale `header.stamp`, to exercise the deadman |
| `emit_episode_events` | `false` | drive the recorder without a human |

```bash
ros2 launch moveit_teleop_bringup teleop.launch.py \
    robot_config_package:=<arm>_moveit_config device:=fake pattern:=circle
```

## `device_base.TeleopDevice`

Base class: owns the two publishers, the `rate_hz` and `frame_id` parameters,
and the `TeleopInput` stamping convention. A new device subclasses it and fills
in a sample.

| `TeleopInput` field | Meaning |
|---|---|
| `pose` | device pose in the device's own convention |
| `pose_valid` | **false** when tracking is lost — not a zero pose. Tracking loss must drop the anchor, and a zero pose is indistinguishable from the origin |
| `clutch` | **level**-triggered, not an event: a dropped release would otherwise leave the arm engaged forever. Level plus timeout fails safe |
| `gripper`, `gripper_valid` | 0…1, normalized; `gripper_valid` false lets a consumer hold the last value rather than slamming to 0. The robot's own open/closed values live in its SRDF |

## Tests

```bash
python3 -m pytest moveit_teleop_devices/test -q
```
