"""Meta Quest teleoperation device over WebXR.

The headset runs a browser, not an app: it loads a page from this node over
HTTPS, opens an immersive-vr session, and streams controller poses and buttons
back over a WebSocket. No Unity, no APK, no ADB, and nothing to rebuild when
the mapping changes.

Control mapping (WebXR "xr-standard" gamepad profile, fixed by that spec):

    middle-finger trigger (squeeze, button 1)  clutch
    index trigger         (button 0, analog)   gripper
    A / X                 (button 4)           toggle recording
    B / Y                 (button 5)           discard last episode

Poses are published RAW in the headset's own frame, exactly as any other
device driver does. Converting WebXR axes (x right, y up, -z forward) to ROS
is the retargeter's job via its ``webxr`` convention -- doing it here would put
the same coordinate logic in two places.

Threading: aiohttp runs its own event loop on a background thread and drops
each frame into a single slot. The ROS timer reads that slot. One shared value
with a lock, no queue: a stale controller pose is worthless, so a backlog would
only add latency.
"""

from __future__ import annotations

import asyncio
import json
import ssl
import threading
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from moveit_teleop_msgs.msg import TeleopEvent
from sensor_msgs.msg import Joy

from .device_base import (
    EVENT_QOS, DeviceSample, TeleopDevice, spin_device,
)


class _Bridge:
    """HTTPS + WebSocket server, run on its own thread."""

    def __init__(self, host, port, certfile, keyfile, web_dir, logger):
        self.host, self.port = host, port
        self.certfile, self.keyfile = certfile, keyfile
        self.web_dir = Path(web_dir)
        self.log = logger

        self._lock = threading.Lock()
        self._latest = None       # most recent decoded frame
        self._clients = 0
        # Open sockets, so haptic feedback can be pushed DOWN to the
        # headset. Everything else in this bridge flows the other way.
        self._sockets = set()
        self._loop = None
        self._loop = None
        self._thread = None
        # The server runs on a daemon thread, so an exception there would
        # otherwise vanish and leave a node that looks healthy and publishes
        # nothing forever. These let start() surface the failure to the caller.
        self._ready = threading.Event()
        self._error = None

    # ---- lifecycle --------------------------------------------------------

    def start(self, timeout_sec: float = 10.0) -> None:
        """Start the server thread and RAISE if it does not come up."""
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout_sec):
            raise RuntimeError(
                f'XR bridge did not start within {timeout_sec}s')
        if self._error is not None:
            raise self._error

    def _run(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._serve())
        except Exception as exc:  # noqa: BLE001
            self._error = exc
            self._ready.set()
            return
        self._ready.set()
        self._loop.run_forever()

    async def _serve(self) -> None:
        # The ROS callback runs on a different thread; sending on an
        # aiohttp socket from there is not safe, so it hands the work to
        # this loop instead.
        import asyncio
        self._loop = asyncio.get_running_loop()
        from aiohttp import web

        app = web.Application()
        app.router.add_get('/', self._index)
        app.router.add_get('/ws', self._websocket)

        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(self.certfile, self.keyfile)

        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, self.host, self.port, ssl_context=context)
        await site.start()
        self._report_reachability()

    def _report_reachability(self) -> None:
        """Print the exact URLs to type, and warn about the usual blocker.

        Binding successfully says nothing about whether the headset can reach
        us. A host firewall silently drops the connection, and the symptom in
        the headset is an indistinguishable "page did not load" -- so say it
        here, where the information is.
        """
        self.log.info(f'XR bridge listening on https://{self.host}:{self.port}')

        addresses = self._local_addresses()

        for addr in addresses:
            self.log.info(f'  headset URL:  https://{addr}:{self.port}')
        if not addresses:
            self.log.info('  (could not determine this host\'s LAN address; '
                          'use `ip -4 addr` and browse to that address)')

        if self._firewall_active():
            example = addresses[0] if addresses else '<this-host-ip>'
            subnet = '.'.join(example.split('.')[:3]) + '.0/24' \
                if example.count('.') == 3 else '<lan>/24'
            self.log.warn(
                'ufw is ACTIVE. If the headset cannot load the page, the '
                'firewall is almost certainly dropping it. Allow the port for '
                'your LAN only:')
            self.log.warn(
                f'    sudo ufw allow from {subnet} to any port {self.port} '
                f"proto tcp comment 'XR teleop'")

    @staticmethod
    def _local_addresses() -> list:
        """This host's non-loopback IPv4 addresses.

        Read from the interfaces rather than by resolving the hostname:
        gethostname() usually maps to 127.0.1.1 on Debian-family systems and
        tells you nothing about which address the headset should dial.
        """
        addresses = []
        try:
            import subprocess
            out = subprocess.run(['ip', '-4', '-o', 'addr', 'show'],
                                 capture_output=True, text=True, timeout=3)
            for line in out.stdout.splitlines():
                parts = line.split()
                if len(parts) < 4 or parts[2] != 'inet':
                    continue
                addr = parts[3].split('/')[0]
                if not addr.startswith('127.'):
                    addresses.append(addr)
        except Exception:  # noqa: BLE001
            pass

        if not addresses:
            # Fallback: ask the kernel which source address it would use.
            # Connecting a UDP socket sends nothing.
            try:
                import socket
                probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                probe.connect(('8.8.8.8', 80))
                addresses = [probe.getsockname()[0]]
                probe.close()
            except Exception:  # noqa: BLE001
                pass
        return addresses

    @staticmethod
    def _firewall_active() -> bool:
        """Whether ufw is actually filtering, not merely installed.

        `systemctl is-active ufw` reports 'active' whenever the unit is
        enabled, which it is on a stock Ubuntu even after `ufw disable` -- the
        unit stays up and applies an empty ruleset. Trusting it sends the
        operator to open a port on a firewall that is not filtering, which is
        the same wasted hunt this warning exists to prevent. `ufw status` is
        authoritative but needs root, so read the flag it sets instead, which
        is world-readable.
        """
        try:
            import subprocess
            out = subprocess.run(['systemctl', 'is-active', 'ufw'],
                                 capture_output=True, text=True, timeout=3)
            if out.stdout.strip() != 'active':
                return False
            with open('/etc/ufw/ufw.conf', encoding='utf-8') as handle:
                return any(line.strip() == 'ENABLED=yes' for line in handle)
        except Exception:  # noqa: BLE001
            return False

    def send_feedback(self, payload: dict) -> None:
        """Push a message to every connected headset, from any thread.

        Called from a ROS subscription callback, which is NOT on the aiohttp
        event loop. Scheduling onto the loop is the only safe way to touch a
        WebSocket from here; writing directly corrupts the connection in ways
        that show up much later as a dropped headset.
        """
        loop = self._loop
        if loop is None or not self._sockets:
            return
        text = json.dumps(payload)

        async def _broadcast():
            for ws in list(self._sockets):
                try:
                    await ws.send_str(text)
                except Exception:                    # noqa: BLE001
                    # A socket that has gone away is not an error worth
                    # logging every frame; the handler removes it.
                    self._sockets.discard(ws)

        try:
            import asyncio
            asyncio.run_coroutine_threadsafe(_broadcast(), loop)
        except RuntimeError:
            pass

    async def _index(self, request):
        from aiohttp import web
        return web.FileResponse(self.web_dir / 'index.html')

    async def _websocket(self, request):
        from aiohttp import web
        ws = web.WebSocketResponse(heartbeat=5.0)
        await ws.prepare(request)
        self._clients += 1
        self._sockets.add(ws)
        self.log.info(f'headset connected ({self._clients} client(s))')
        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    try:
                        frame = json.loads(msg.data)
                    except json.JSONDecodeError:
                        continue
                    with self._lock:
                        self._latest = frame
        finally:
            self._sockets.discard(ws)
            self._clients -= 1
            self.log.warn('headset disconnected')
            # Deliberately NOT clearing _latest here. The node detects the
            # stall by the headset timestamp ceasing to advance, which also
            # covers a socket that stays open while the headset freezes -- a
            # disconnect is only one of the ways input can die.
        return ws

    def take(self):
        with self._lock:
            return self._latest

    @property
    def connected(self) -> bool:
        return self._clients > 0


class XRDevice(TeleopDevice):
    """Publishes Quest controller state as normalized teleop input."""

    def __init__(self, **node_kwargs):
        super().__init__('xr_device', default_rate_hz=90.0,
                         default_frame_id='webxr_local_floor', **node_kwargs)

        self.declare_parameter('hand', 'right')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8443)
        self.declare_parameter('certfile', '')
        self.declare_parameter('keyfile', '')
        self.declare_parameter('web_dir', '')
        # Below this the trigger is treated as fully released. Quest triggers
        # rest a little above zero, and without a floor the gripper creeps.
        self.declare_parameter('gripper_deadzone', 0.03)
        self.declare_parameter('clutch_threshold', 0.5)
        # Haptic feedback for virtual-wall contact. Teleoperating against
        # a surface you cannot feel is the hard part: the tool is sliding
        # along a wall and your hand is somewhere below it with nothing
        # to say so.
        self.declare_parameter('feedback_topic', '/teleop/feedback')
        self.declare_parameter('haptics_enabled', True)

        self.hand = str(self.get_parameter('hand').value).lower()
        if self.hand not in ('left', 'right'):
            raise ValueError(f"hand must be 'left' or 'right', got {self.hand!r}")
        self.gripper_deadzone = float(self.get_parameter('gripper_deadzone').value)
        self.clutch_threshold = float(self.get_parameter('clutch_threshold').value)

        web_dir = self.get_parameter('web_dir').value or str(
            Path(get_package_share_directory('moveit_teleop_devices')) / 'web')

        certfile = self.get_parameter('certfile').value
        keyfile = self.get_parameter('keyfile').value
        if not certfile or not keyfile:
            raise RuntimeError(
                'certfile and keyfile are required: WebXR only runs in a secure '
                'context, so the page must be served over HTTPS. Generate a '
                'self-signed pair with make_cert.sh.')

        # Checked here rather than left to load_cert_chain, whose
        # FileNotFoundError is an OSError and so is caught below and reported
        # as a port clash -- sending you after a stale process that does not
        # exist. This is now the first thing a fresh host hits, because the
        # device defaults to 'xr'.
        absent = [f for f in (certfile, keyfile) if not Path(f).is_file()]
        if absent:
            raise RuntimeError(
                f"no certificate at {' and '.join(absent)}. Generate one, "
                'once per host, with the LAN address the headset will browse '
                'to:\n'
                '    bash $(ros2 pkg prefix moveit_teleop_devices)/share/'
                'moveit_teleop_devices/scripts/make_cert.sh <this-host-ip>')

        if self.get_parameter('haptics_enabled').value:
            from moveit_teleop_msgs.msg import TeleopFeedback
            self.create_subscription(
                TeleopFeedback,
                self.get_parameter('feedback_topic').value,
                self._on_feedback, EVENT_QOS)

        self._bridge = _Bridge(
            self.get_parameter('host').value,
            int(self.get_parameter('port').value),
            certfile, keyfile, web_dir, self.get_logger())
        try:
            self._bridge.start()
        except OSError as exc:
            # Almost always a leftover xr_device from a previous run. Say so:
            # "address already in use" on its own sends people hunting for a
            # firewall problem.
            raise RuntimeError(
                f'could not start the XR bridge on '
                f"{self.get_parameter('host').value}:"
                f"{self.get_parameter('port').value}: {exc}. "
                'If this is "address already in use", another xr_device is '
                'still running: pkill -f xr_device') from exc

        self._joy_pub = self.create_publisher(Joy, 'thumbsticks', 10)

        self._last_headset_t = None
        self._prev = {}          # button edge detection
        self._recording = False
        self._warned_missing = False

        self.get_logger().info(
            f"driving from the {self.hand} controller; clutch = middle-finger "
            f'trigger, gripper = index trigger, A/X = record toggle, '
            f'B/Y = discard last, thumbstick = home')

    # ---- device interface -------------------------------------------------

    def _on_feedback(self, msg) -> None:
        """Forward wall contact to the headset as a buzz.

        The message is a LEVEL, so this simply mirrors it: the page starts a
        continuous rumble on `active` and stops on its absence. Nothing here
        decides how long to buzz, which is what makes a dropped message
        harmless rather than leaving the controller running.
        """
        self._bridge.send_feedback({
            'type': 'haptic',
            'active': bool(msg.active),
            'intensity': float(msg.intensity),
            'duration': float(msg.duration),
            'sources': list(msg.sources),
            'depth': float(msg.depth),
        })

    def poll(self):
        frame = self._bridge.take()
        if frame is None:
            return None

        headset_t = frame.get('t')
        if headset_t is None or headset_t == self._last_headset_t:
            # The headset clock has not advanced, so this is the same frame we
            # already published. Returning None rather than republishing is
            # what lets the watchdog see a frozen headset: a driver that keeps
            # re-emitting a stale pose looks perfectly healthy.
            return None
        self._last_headset_t = headset_t

        all_controllers = frame.get('controllers') or {}
        left_c = all_controllers.get('left')
        right_c = all_controllers.get('right')
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = [
            float(left_c.get('sx', 0.0)) if left_c else 0.0,
            float(left_c.get('sy', 0.0)) if left_c else 0.0,
            float(right_c.get('sx', 0.0)) if right_c else 0.0,
            float(right_c.get('sy', 0.0)) if right_c else 0.0,
        ]
        self._joy_pub.publish(joy_msg)

        controller = all_controllers.get(self.hand)
        if controller is None:
            if not self._warned_missing:
                present = list((frame.get('controllers') or {}).keys())
                self.get_logger().warn(
                    f'{self.hand} controller not reported (present: {present}); '
                    'is it awake?')
                self._warned_missing = True
            return DeviceSample(pose_valid=False, clutch=False, gripper_valid=False)
        self._warned_missing = False

        events = self._events(controller)

        trigger = float(controller.get('trigger', 0.0))
        gripper = 0.0 if trigger < self.gripper_deadzone else trigger

        return DeviceSample(
            position=tuple(controller.get('p', (0.0, 0.0, 0.0))),
            orientation=tuple(controller.get('q', (0.0, 0.0, 0.0, 1.0))),
            pose_valid=bool(controller.get('valid', False)),
            clutch=float(controller.get('squeeze', 0.0)) >= self.clutch_threshold,
            gripper=gripper,
            gripper_valid=True,
            events=events,
            # Stamped with ROS time by the base class. The headset's clock is
            # unrelated to this machine's, so using it here would make the
            # watchdog measure clock skew rather than staleness.
            stamp=None,
        )

    def _events(self, controller) -> list:
        """Turn button levels into edge-triggered events."""
        events = []
        for name, handler in (('a', self._on_a), ('b', self._on_b),
                              ('stick', self._on_stick)):
            pressed = bool(controller.get(name, False))
            if pressed and not self._prev.get(name, False):
                handler(events)
            self._prev[name] = pressed
        return events

    def _on_a(self, events) -> None:
        self._recording = not self._recording
        if self._recording:
            events.append((TeleopEvent.RECORD_START, ''))
            self.get_logger().info('A: recording STARTED')
        else:
            events.append((TeleopEvent.RECORD_STOP, ''))
            events.append((TeleopEvent.EPISODE_SAVE, ''))
            self.get_logger().info('A: recording STOPPED and saved')

    def _on_stick(self, events) -> None:
        # Thumbstick CLICK, not a nudge: homing sweeps the arm across the
        # workspace, so it wants a button you cannot brush by accident while
        # steering. The mode manager refuses it while the clutch is held.
        events.append((TeleopEvent.HOME, ''))
        self.get_logger().info('thumbstick: HOME (return to ready pose)')

    def _on_b(self, events) -> None:
        if self._recording:
            # Mid-take B means "throw this one away", which also ends it --
            # otherwise the operator is left recording a take they just
            # rejected.
            self._recording = False
        events.append((TeleopEvent.EPISODE_DISCARD, ''))
        self.get_logger().info('B: discard last episode')


def main(args=None):
    spin_device(XRDevice, args=args)


if __name__ == '__main__':
    main()
