#!/usr/bin/env python3
"""Measure how well the arm tracks a trajectory, so "smooth" becomes a number.

Every smoothness change so far has been judged by eye, which is why the wrong
cause survived three rounds. This sends one trajectory, records /joint_states
throughout, and reports metrics that distinguish the candidate causes from each
other:

    source /opt/ros/jazzy/setup.bash
    python3 measure_tracking.py --joint 4 --degrees -15 --yes-move
    python3 measure_tracking.py --joint 4 --degrees -15 --yes-move --csv run.csv
    python3 measure_tracking.py --joint 4 --degrees -15 --yes-move --plot run.html

--plot writes a self-contained HTML page (no matplotlib, nothing to install)
with every joint's speed over time, so cross-talk into joints that were not
commanded is visible alongside the commanded one. Open it in any browser, or
scp it to a machine that has one.

READ THE REPEATED-SAMPLE FIGURE FIRST. /joint_states is published at
publish_rate_hz, but the value it carries only changes when the poll thread has
a fresh reading. When a fifth to a third of consecutive samples repeat, the
finest wiggles in any velocity computed from them are the measurement, not the
arm: a repeat gives velocity zero, the next sample gives a double-size step, and
that pair looks exactly like a stutter. This script therefore differentiates
over a window wide enough to span the repeats, and tells you what fraction
repeated so the fine detail can be discounted honestly.

WHAT IT REPORTS, AND WHAT EACH ONE MEANS

  tracking error (from the action's own feedback: desired vs actual)
      RMS and peak lag between where the driver said to be and where the arm
      was. Large and roughly constant => the controller is trailing the
      setpoint, which is what low stiffness does. Large only at the ends =>
      acceleration limits.

  acceleration sign reversals
      The roughness metric. A single smooth move accelerates, coasts and
      decelerates, so its acceleration goes positive, through zero, negative:
      ONE reversal. (The coast is ignored -- values under 10% of peak are
      treated as zero so sensor noise is not counted, which is also why a clean
      trapezoid scores 1 rather than 2.) Every additional reversal is a
      velocity wobble you would feel as a stutter. This is the number that
      should fall when re-plan churn is reduced -- see
      trajectory_send_rate_hz.

  peak |velocity| / |acceleration|
      Compared against what the trajectory asked for. Well above it means the
      controller is overshooting each setpoint and being pulled back, i.e.
      default_max_joint_vel/acc are far above the trajectory's own profile.

  final error and settle time
      Accuracy. In impedance mode expect a steady residual equal to
      (unmodelled torque / K_q) -- gravity sag from an undeclared tool. That
      one does not improve with tuning; it improves by declaring the tool.

CONFIGURATIONS WORTH COMPARING (same move each time)

  joint_control_mode:=position                   vs impedance
  joint_stiffness_ratio 1.0                      vs 0.3
  trajectory_send_rate_hz 50                     vs 10
  default_max_joint_acc 3.0                      vs 0.5

Change one at a time with `ros2 param set` and re-run. --csv writes the raw
samples so two runs can be plotted against each other.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrackingRun(Node):
    def __init__(self, namespace, joint_names):
        super().__init__("measure_tracking")
        self._ns = namespace.strip("/")
        self._joint_names = joint_names
        self._samples = []          # (t, {joint: position})
        self._feedback = []         # (t, desired, actual)
        self._latest = {}
        self.create_subscription(JointState, "/joint_states", self._on_js,
                                 qos_profile_sensor_data)
        self._client = ActionClient(
            self, FollowJointTrajectory,
            f"/{self._ns}/follow_joint_trajectory" if self._ns
            else "/follow_joint_trajectory")
        self._recording = False
        self._t0 = 0.0

    def _on_js(self, msg):
        for name, position in zip(msg.name, msg.position):
            self._latest[name] = position
        if self._recording and all(n in self._latest for n in self._joint_names):
            self._samples.append((
                time.monotonic() - self._t0,
                {n: self._latest[n] for n in self._joint_names},
            ))

    def _on_feedback(self, msg):
        if not self._recording:
            return
        fb = msg.feedback
        self._feedback.append((
            time.monotonic() - self._t0,
            list(fb.desired.positions),
            list(fb.actual.positions),
        ))

    def wait_for_state(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(n in self._latest for n in self._joint_names):
                return {n: self._latest[n] for n in self._joint_names}
        raise TimeoutError("no /joint_states for all arm joints")

    def run(self, index, degrees, duration, timeout):
        start = self.wait_for_state()
        name = self._joint_names[index]
        target = dict(start)
        target[name] = start[name] + math.radians(degrees)

        if not self._client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("no follow_joint_trajectory server")

        traj = JointTrajectory()
        traj.joint_names = list(self._joint_names)
        p0 = JointTrajectoryPoint()
        p0.positions = [start[n] for n in self._joint_names]
        p0.time_from_start = Duration(sec=0, nanosec=0)
        p1 = JointTrajectoryPoint()
        p1.positions = [target[n] for n in self._joint_names]
        p1.time_from_start = Duration(sec=int(duration),
                                      nanosec=int((duration % 1) * 1e9))
        traj.points = [p0, p1]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        print(f"moving {name} {math.degrees(start[name]):.2f}° -> "
              f"{math.degrees(target[name]):.2f}° over {duration:.1f}s\n")
        self._t0 = time.monotonic()
        self._recording = True
        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("goal rejected — run check_arm_ros.py")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=duration + timeout + 30.0)
        # Keep recording briefly after the goal ends to catch settling / sag.
        settle_until = time.monotonic() + 1.5
        while time.monotonic() < settle_until:
            rclpy.spin_once(self, timeout_sec=0.02)
        self._recording = False

        wrapped = result_future.result()
        return (wrapped.result if wrapped else None,
                wrapped.status if wrapped else None,
                name, target[name])


def _repeat_fraction(values):
    """Fraction of consecutive samples identical to their predecessor."""
    if len(values) < 2:
        return 0.0
    repeats = sum(1 for i in range(1, len(values)) if values[i] == values[i - 1])
    return repeats / (len(values) - 1)


def _smooth_velocity(times, values, half_window):
    """Central difference over +/-half_window samples.

    A one-sample difference is useless here: with repeated samples it alternates
    between zero and a double step. The window has to span the repeats.
    """
    out = []
    for i in range(half_window, len(values) - half_window):
        dt = times[i + half_window] - times[i - half_window]
        if dt > 0:
            out.append((times[i],
                        (values[i + half_window] - values[i - half_window]) / dt))
    return out


def _derivatives(times, values):
    """Finite-difference velocity and acceleration, unevenly sampled."""
    vel, acc = [], []
    for i in range(1, len(values)):
        dt = times[i] - times[i - 1]
        vel.append((values[i] - values[i - 1]) / dt if dt > 0 else 0.0)
    for i in range(1, len(vel)):
        dt = times[i + 1] - times[i]
        acc.append((vel[i] - vel[i - 1]) / dt if dt > 0 else 0.0)
    return vel, acc


def _sign_reversals(series, deadband):
    """Sign changes, ignoring values inside a deadband so noise is not counted."""
    reversals, last = 0, 0
    for value in series:
        sign = 0 if abs(value) < deadband else (1 if value > 0 else -1)
        if sign and last and sign != last:
            reversals += 1
        if sign:
            last = sign
    return reversals


_PLOT_TEMPLATE = """<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Joint speeds</title><style>
:root{color-scheme:light;--s0:#f6f6f4;--s1:#fcfcfb;--bd:#dddcd6;--gr:#e8e7e1;
--tp:#0b0b0b;--ts:#52514e;--tm:#7a7973}
@media(prefers-color-scheme:dark){:root:not([data-theme="light"]){color-scheme:dark;
--s0:#121211;--s1:#1a1a19;--bd:#35342f;--gr:#2a2a26;--tp:#fff;--ts:#c3c2b7;--tm:#8f8e84}}
:root[data-theme="dark"]{color-scheme:dark;--s0:#121211;--s1:#1a1a19;--bd:#35342f;
--gr:#2a2a26;--tp:#fff;--ts:#c3c2b7;--tm:#8f8e84}
*{box-sizing:border-box}body{margin:0;padding:26px 16px 44px;background:var(--s0);
color:var(--tp);font:15px/1.55 ui-sans-serif,system-ui,-apple-system,sans-serif}
.wrap{max-width:900px;margin:0 auto}h1{font-size:20px;margin:0 0 6px}
.sub{color:var(--ts);font-size:13.5px;margin:0 0 20px}
.card{background:var(--s1);border:1px solid var(--bd);border-radius:10px;
padding:18px 18px 10px;margin-bottom:18px}
.lg{display:flex;flex-wrap:wrap;gap:14px;margin:0 0 12px}
.lg div{display:flex;align-items:center;gap:6px;font-size:12.5px;color:var(--ts)}
.sw{width:13px;height:3px;border-radius:2px;flex:none}
svg{display:block;width:100%;height:auto;overflow:visible}
.g{stroke:var(--gr);stroke-width:1}.ax{stroke:var(--bd);stroke-width:1}
.tk{fill:var(--tm);font-size:11px}.at{fill:var(--ts);font-size:12px}
.sr{fill:none;stroke-width:2;stroke-linejoin:round;stroke-linecap:round}
table{border-collapse:collapse;width:100%;font-size:13px;font-variant-numeric:tabular-nums}
caption{text-align:left;color:var(--ts);font-size:13px;padding:0 0 10px}
th,td{text-align:right;padding:6px 10px;border-bottom:1px solid var(--bd)}
th:first-child,td:first-child{text-align:left}th{color:var(--ts);font-weight:600}
.note{color:var(--ts);font-size:13.5px}.note b{color:var(--tp)}
</style></head><body><div class="wrap">
<h1>__TITLE__</h1><p class="sub">__SUB__</p>
<div class="card"><div class="lg" id="lg"></div>
<svg id="c" viewBox="0 0 900 380" role="img" aria-label="joint speed over time"></svg></div>
<div class="card"><table><caption>Peak speed per joint. Only the commanded joint
should be far from zero; anything else is cross-talk.</caption>
<thead><tr><th>Joint</th><th>Peak speed</th><th>Travelled</th></tr></thead>
<tbody id="tb"></tbody></table></div>
<p class="note"><b>Repeated samples: __DUP__%.</b> Speed is a central difference
over __WIN__ ms, wide enough to span them. Trust the envelope, not single spikes.</p>
</div><script>
const D=__DATA__;
const C=['#2a78d6','#eb6834','#1baf7a','#eda100','#e87ba4','#008300','#4a3aa7'];
const W=900,H=380,M={t:18,r:86,b:46,l:56},iw=W-M.l-M.r,ih=H-M.t-M.b;
const xM=Math.max(...D.series.flatMap(s=>s.pts.map(p=>p[0])))||1;
const yM=Math.max(1,Math.ceil(Math.max(...D.series.flatMap(s=>s.pts.map(p=>Math.abs(p[1]))))));
const X=v=>M.l+v/xM*iw,Y=v=>M.t+ih-v/yM*ih;
const ns='http://www.w3.org/2000/svg',sv=document.getElementById('c');
const E=(n,a)=>{const e=document.createElementNS(ns,n);for(const k in a)e.setAttribute(k,a[k]);return e};
for(let i=0;i<=5;i++){const v=yM/5*i,y=Y(v);
sv.appendChild(E('line',{class:'g',x1:M.l,x2:M.l+iw,y1:y,y2:y}));
const t=E('text',{class:'tk',x:M.l-9,y:y+4,'text-anchor':'end'});t.textContent=v.toFixed(1);sv.appendChild(t)}
sv.appendChild(E('line',{class:'ax',x1:M.l,x2:M.l+iw,y1:M.t+ih,y2:M.t+ih}));
for(let v=0;v<=xM+1e-9;v+=1){const t=E('text',{class:'tk',x:X(v),y:M.t+ih+18,'text-anchor':'middle'});
t.textContent=v.toFixed(0)+'s';sv.appendChild(t)}
let a=E('text',{class:'at',x:M.l+iw/2,y:H-6,'text-anchor':'middle'});a.textContent='time from start of motion';sv.appendChild(a);
a=E('text',{class:'at',x:M.l-42,y:M.t+ih/2,'text-anchor':'middle',transform:`rotate(-90 ${M.l-42} ${M.t+ih/2})`});
a.textContent='speed (deg/s)';sv.appendChild(a);
D.series.forEach((s,i)=>{const col=C[i%C.length];
sv.appendChild(E('path',{class:'sr',d:s.pts.map((p,j)=>(j?'L':'M')+X(p[0]).toFixed(1)+' '+Y(Math.abs(p[1])).toFixed(1)).join(' '),stroke:col}));
if(s.commanded){const l=E('text',{x:M.l+iw+8,y:Y(Math.abs(s.pts[s.pts.length-1][1]))+4,fill:col,'font-size':'12','font-weight':'600'});
l.textContent=s.name.replace(/^(Left|Right)_/,'');sv.appendChild(l)}});
document.getElementById('lg').innerHTML=D.series.map((s,i)=>
`<div><span class="sw" style="background:${C[i%C.length]}"></span>${s.name}${s.commanded?' (commanded)':''}</div>`).join('');
document.getElementById('tb').innerHTML=D.series.map(s=>
`<tr><td>${s.name}${s.commanded?' <b>(commanded)</b>':''}</td><td>${Math.max(...s.pts.map(p=>Math.abs(p[1]))).toFixed(2)} deg/s</td><td>${s.travelled.toFixed(2)}&deg;</td></tr>`).join('');
</script></body></html>"""


def write_plot(path, run, commanded, half_window, dup_pct):
    """Self-contained HTML: every joint's speed over time. No dependencies."""
    samples = run._samples
    times = [t for t, _ in samples]
    t0 = times[half_window] if len(times) > half_window else times[0]
    series = []
    for jname in run._joint_names:
        values = [q[jname] for _, q in samples]
        vel = _smooth_velocity(times, values, half_window)
        series.append({
            "name": jname,
            "commanded": jname == commanded,
            "travelled": math.degrees(values[-1] - values[0]),
            "pts": [[round(t - t0, 3), round(math.degrees(v), 3)] for t, v in vel],
        })
    window_ms = 2 * half_window * (times[-1] - times[0]) / max(len(times) - 1, 1) * 1000
    html = (_PLOT_TEMPLATE
            .replace("__TITLE__", f"Joint speeds during a move of {commanded}")
            .replace("__SUB__", "Every joint in the group, so cross-talk into "
                                "joints that were not commanded is visible.")
            .replace("__DUP__", f"{dup_pct:.0f}")
            .replace("__WIN__", f"{window_ms:.0f}")
            .replace("__DATA__", json.dumps({"series": series},
                                            separators=(",", ":"))))
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(html)
    return path


def report(run, name, target, result, status, csv_path, plot_path=None,
           half_window=3):
    samples, feedback = run._samples, run._feedback
    if len(samples) < 5:
        print(f"only {len(samples)} samples recorded — nothing to analyse")
        return 1

    times = [t for t, _ in samples]
    values = [q[name] for _, q in samples]
    vel, acc = _derivatives(times, values)
    span = times[-1] - times[0]
    dup_pct = 100.0 * _repeat_fraction(values)

    print("=" * 66)
    print(f"MOTION OF {name}")
    print("=" * 66)
    print(f"  samples            {len(samples)} over {span:.2f}s "
          f"({len(samples) / span:.1f} Hz)")
    print(f"  repeated samples   {dup_pct:.0f}%   "
          f"(identical to the previous one; above ~10% the fine detail below "
          f"is measurement, not motion)")
    print(f"  travelled          {math.degrees(values[-1] - values[0]):+.2f}°")
    print(f"  peak |velocity|    {math.degrees(max(abs(v) for v in vel)):.1f} °/s")
    if acc:
        print(f"  peak |accel|       "
              f"{math.degrees(max(abs(a) for a in acc)):.1f} °/s²")
        # 10% of peak keeps sensor noise from counting as a reversal.
        deadband = 0.1 * max(abs(a) for a in acc)
        reversals = _sign_reversals(acc, deadband)
        print(f"  accel reversals    {reversals}   "
              f"(1 = one smooth accelerate-coast-decelerate; "
              f"each extra one is a stutter)")
        if dup_pct > 10.0:
            print(f"                     ^ inflated by the {dup_pct:.0f}% "
                  f"repeated samples — compare runs, do not read absolutely")
        # The honest number: differentiate over a window that spans the repeats.
        # Reported at two widths, because that is what separates real stepping
        # from sampling noise: noise shrinks as the window grows, real stepping
        # does not. Comparing a single width between runs at different speeds
        # is misleading -- the same absolute sampling noise is a smaller
        # fraction of a faster cruise.
        def _spread(hw):
            smooth = _smooth_velocity(times, values, hw)
            if len(smooth) <= 3:
                return None
            speeds = [abs(v) for _, v in smooth]
            peak = max(speeds)
            cruise = [v for v in speeds if v > 0.5 * peak]
            mean = sum(cruise) / len(cruise)
            return (math.degrees(peak), math.degrees(mean),
                    (max(cruise) - min(cruise)) / mean * 100.0)

        narrow = _spread(half_window)
        wide = _spread(half_window * 3)
        if narrow:
            sample_dt = span / max(len(times) - 1, 1)
            print(f"  smoothed peak      {narrow[0]:.1f} °/s")
            print(f"  mean cruise speed  {narrow[1]:.1f} °/s   "
                  f"(compare spreads only between runs at a similar speed)")
            print(f"  cruise spread      {narrow[2]:.0f}% over "
                  f"{2 * half_window * sample_dt * 1000:.0f}ms", end="")
            if wide:
                print(f"   -> {wide[2]:.0f}% over "
                      f"{6 * half_window * sample_dt * 1000:.0f}ms")
                drop = narrow[2] - wide[2]
                if drop > 15.0:
                    print(f"                     the {drop:.0f}-point drop with "
                          f"a wider window means much of the narrow figure is "
                          f"sampling noise; {wide[2]:.0f}% is the real stepping")
                elif dup_pct > 20.0:
                    # Wrong to call this real stepping. Corruption affecting
                    # one sample in three does not wash out of a window only
                    # three times wider, so a spread that survives says
                    # nothing either way while the repeat rate is this high.
                    print(f"                     with {dup_pct:.0f}% repeated "
                          f"samples a spread that survives a 3x window is NOT "
                          f"evidence of real stepping — the corruption is too "
                          f"dense to wash out. Fix acquisition before reading "
                          f"this number at all.")
                else:
                    print(f"                     holding up across windows "
                          f"means this is real stepping, not sampling noise")
            else:
                print()

    print()
    print("=" * 66)
    print("TRACKING (driver's own desired vs actual)")
    print("=" * 66)
    if not feedback:
        print("  no feedback received — the driver publishes it per send, so")
        print("  either the move was too short or feedback is not reaching us")
    else:
        idx = run._joint_names.index(name)
        errors = [abs(d[idx] - a[idx]) for _, d, a in feedback
                  if len(d) > idx and len(a) > idx]
        if errors:
            rms = math.sqrt(sum(e * e for e in errors) / len(errors))
            print(f"  feedback samples   {len(errors)} "
                  f"({len(errors) / span:.1f} Hz)")
            print(f"  RMS lag            {math.degrees(rms):.3f}°")
            print(f"  peak lag           {math.degrees(max(errors)):.3f}°")
            print(f"  median lag         "
                  f"{math.degrees(statistics.median(errors)):.3f}°")

    print()
    print("=" * 66)
    print("ACCURACY")
    print("=" * 66)
    final = values[-1]
    print(f"  final              {math.degrees(final):.3f}°")
    print(f"  target             {math.degrees(target):.3f}°")
    print(f"  error              {math.degrees(final - target):+.3f}°")
    # Drift over the post-goal window is sag, not tracking.
    tail = [q[name] for t, q in samples if t > times[-1] - 1.0]
    if len(tail) > 2:
        print(f"  drift in last 1s   "
              f"{math.degrees(max(tail) - min(tail)):.3f}°  "
              f"(a steady residual here is gravity sag, not tracking)")
    if result is not None:
        print(f"  error_code         {result.error_code}"
              + (f", {result.error_string!r}" if result.error_string else ""))
    print(f"  goal status        {status}")

    if plot_path:
        write_plot(plot_path, run, name, half_window, dup_pct)
        print(f"\nwrote {plot_path} — open it in a browser")

    if csv_path:
        with open(csv_path, "w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(["t", "position_rad", "position_deg",
                             "velocity_deg_s", "accel_deg_s2"])
            for i, (t, q) in enumerate(samples):
                v = math.degrees(vel[i - 1]) if 0 < i <= len(vel) else ""
                a = math.degrees(acc[i - 2]) if 1 < i <= len(acc) + 1 else ""
                writer.writerow([f"{t:.4f}", f"{q[name]:.6f}",
                                 f"{math.degrees(q[name]):.4f}", v, a])
        print(f"\nwrote {csv_path}")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--namespace", default="left_arm")
    ap.add_argument("--joint", type=int, default=4)
    ap.add_argument("--degrees", type=float, default=-15.0)
    ap.add_argument("--duration", type=float, default=4.0)
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--csv", help="write the raw samples here")
    ap.add_argument("--plot", help="write a self-contained HTML plot of every "
                                   "joint's speed here")
    ap.add_argument("--smooth-window", type=int, default=3,
                    help="half-width in samples for the smoothed velocity; "
                         "must span the repeated samples")
    ap.add_argument("--yes-move", action="store_true", help="required: it moves")
    args = ap.parse_args()

    if not args.yes_move:
        print("This moves the arm. Add --yes-move.")
        return 0
    if not 1 <= args.joint <= 7:
        raise SystemExit("--joint must be 1..7")

    prefix = "Right" if "right" in args.namespace.lower() else "Left"
    joint_names = [f"{prefix}_joint{i}" for i in range(1, 8)]

    rclpy.init()
    run = TrackingRun(args.namespace, joint_names)
    try:
        result, status, name, target = run.run(
            args.joint - 1, args.degrees, args.duration, args.timeout)
        return report(run, name, target, result, status, args.csv,
                      args.plot, args.smooth_window)
    except (RuntimeError, TimeoutError) as exc:
        print(f"\nFAILED: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        run.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
