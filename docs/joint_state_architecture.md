# Joint State Pipeline — Architecture Design

Target: replace the current 50 Hz, publish-time-stamped joint state path with a
decoupled acquisition/publication design that uses the RDK's 1 kHz stream and its
hardware timestamps, and exposes the waist joints that are currently discarded.

Everything below is grounded in measurements taken on the hardware on 2026-08-31.
Where something is unverified it says so.

---

## 1. Measured facts this design rests on

| Fact | Value | How it was established |
|---|---|---|
| RDK state refresh | 1000 Hz | 9,998 distinct `q` vectors in 10 s of tight polling |
| Device timestamp cadence | 1000.2 µs ± 19.1 µs | `diff` of `RobotStates.timestamp` over the same capture |
| `states()` call cost | non-blocking, cached | 29.2 M polls in 120 s returned 119,981 unique samples |
| `RobotStates.q` length | **9**, not 7 | `q[0]`=waist yaw, `q[1]`=waist pitch, `q[2:9]`=that arm's joints |
| Both arms report same waist | mean diff 0.000/0.001 µrad, σ 0.014/0.020 | simultaneous sampling of both controllers |
| Two RDK sessions to one arm | **works** | process A held 30 s while B connected, read, and disconnected; A unaffected |
| Transport | UDP/IPv4, arm:65527 → host:9917 and :9925 | tcpdump with `-e`, no EtherCAT frames present |
| Arm joint noise (static) | σ 0.89–1.61 µrad, LSB 0.060–0.238 | 333 s at 50 Hz, 16,650 samples |
| Waist joint noise (static) | σ 0.014/0.019 µrad, LSB 0.0185/0.0149 | 120 s at 1 kHz, 119,981 samples, 10 discrete levels each |
| Host clock offset (left arm) | 29107.53 s, σ 291 µs | 100 samples vs `time.time()` |
| Host clock offset (right arm) | 29120.53 s, σ 287 µs | same method |
| **Inter-arm clock disagreement** | **13.00 s** | difference of the two above |
| Offset drift | ~171 ms over 40 min (~70 ppm) | two measurements of the left arm |
| Host PTP capability | hardware tx/rx, PHC0 live on `enP2p1s0` | `ethtool -T`, `phc_ctl` |
| PTP traffic on segment | none | 30 s tcpdump on 0x88f7 / udp 319 / udp 320 |

Two caveats that matter:

- **All noise figures are static.** Nothing here characterises motion. At 1 rad/s
  a 1 ms timing error is 1000 µrad, which dwarfs the ~1 µrad read noise. Dynamic
  behaviour (backlash, flex, servo lag) is unmeasured.
- **The 291 µs offset jitter is an upper bound**, not the true clock jitter. It
  was measured by sampling at 20–50 Hz against a 1 kHz source, so up to 1 ms of
  phase quantisation is folded in. See §6 for how to do it properly.

---

## 2. What exists today, and why it needs to change

```
                    ┌──────────────────────┐
  Rizon left ──────►│ left_arm_driver      │──► /left_arm/joint_states  (7 joints)
  (192.168.1.100)   │  timer @ 50 Hz       │
                    │  stamp = now()       │
                    └──────────────────────┘
                                                                ┌──────────────────┐
                    ┌──────────────────────┐                    │ joint_state_     │
  Rizon right ─────►│ right_arm_driver     │──► /right_arm/...  │ merger           │
  (192.168.1.101)   │  timer @ 50 Hz       │     (7 joints)     │  waist ≡ 0.0     │
                    │  stamp = now()       │                    └────────┬─────────┘
                    └──────────────────────┘                             │
                                                                         ▼
                                                              /joint_states (16)
                                                                         │
                                                                         ▼
                                                            robot_state_publisher → TF
```

Three structural problems, worst first:

**(a) Timer-driven sampling.** A `create_timer(1/50)` decides *when* to read.
`states()` returns whatever the cached snapshot happens to be at that instant, so
this is aperiodic sampling of a 1 kHz signal — not decimation. Anything above
25 Hz aliases into the output rather than being filtered out. Structural
resonances typically live right there.

**(b) Publish-time stamping.** `arm_driver_node.py:668` sets
`stamp = self.get_clock().now()`. That is when the ROS callback ran, not when the
encoder was read. Invisible at rest; at 1 rad/s it is 1000 µrad per millisecond
of latency. The RDK hands us a hardware timestamp and we throw it away.

**(c) The merger adds a hop and a second resampling.** It waits for a message
from each arm, so its output is as stale as the slower one, then republishes on
its own timer. Two arms' data taken at different instants get one shared stamp.

**(d) Waist joints hardcoded to zero.** `joint_state_merger` publishes
`AGV_Joint1` and `AGV_Joint2` as 0.0 while real 1 kHz data sits unused in
`q[0]`/`q[1]`. Every TF consumer downstream — including anything relating the
base LiDAR to the head camera — is working from a waist pose that is fiction.

---

## 3. Target architecture

```
                    ┌────────────────────────────────────────┐
  Rizon left ──────►│ left_arm_driver                        │
  (.100)            │  ┌──────────────┐   ┌───────────────┐  │
         ▲          │  │ poll thread  │──►│ latest-sample │──┼──► /joint_states
         │          │  │  dedupe on   │   │ slot (atomic) │  │    (7 arm joints,
         │          │  │  timestamp   │   └───────┬───────┘  │     device stamp)
         │          │  └──────────────┘           │          │
         │          │  ┌──────────────┐    publish timer     │
         │          │  │ offset       │    @ publish_rate_hz │
         │          │  │ estimator    │───────────┘          │
         │          │  └──────────────┘                      │
         │          └────────────────────────────────────────┘
         │
         │          ┌────────────────────────────────────────┐
         └──────────│ waist_driver         (2nd RDK session) │──► /joint_states
                    │  same structure, publishes q[0], q[1]  │    (2 waist joints,
                    └────────────────────────────────────────┘     device stamp)

                    ┌────────────────────────────────────────┐
  Rizon right ─────►│ right_arm_driver                       │──► /joint_states
  (.101)            │  same structure                        │    (7 arm joints)
                    └────────────────────────────────────────┘
                                                                        │
                                                                        ▼
                                                           robot_state_publisher → TF
```

Three publishers, one topic, no merger. Each node publishes only the joints it
owns, with its own honest timestamp. One physical joint, one publisher.

> **THIS DESIGN DOES NOT WORK AS WRITTEN — 2026-09-26.** It rests on
> `robot_state_publisher` merging partial `JointState` messages by joint name.
> It does not. In `robot_state_publisher.cpp` (Jazzy), `callbackJointState`
> declares its map as a **local**:
>
> ```cpp
> std::map<std::string, double> joint_positions;          // line 343, local
> for (size_t i = 0; i < state->name.size(); ++i)
>   joint_positions.insert({state->name[i], state->position[i]});
> publishTransforms(joint_positions, state->header.stamp);
> ```
>
> The map is rebuilt from scratch on every message, so a message publishes
> transforms for **only the joints it contains**. Nothing accumulates across
> messages.
>
> Observed consequence with `direct_joint_states:=true` and no waist publisher:
> `AGV_Joint1`/`AGV_Joint2` never appear in any message, so their transforms are
> never published. Both arms hang off `AGV_Pitch` via those two revolute joints,
> so the entire upper body loses its transform chain from `base_link` and RViz
> renders both arms detached from the chassis.
>
> This is also why the waist driver "races last-writer-wins" with the merger
> (sec 8 step 3): at 200 Hz it publishes 2-joint messages while the merger
> publishes 16-joint messages at 50 Hz, and because nothing accumulates, each
> message alone decides what gets a transform that instant.
>
> **Until one publisher emits all 16 joints in a single message, the merger has
> to stay.** The path to removing it is not "let RSP merge" — it is to give the
> single publisher the waist values. `arm_driver_node._extract_arm_q` already
> reads the full 9-DoF `q` off the left arm's existing RDK session and discards
> indices 0-1, which are exactly `AGV_Joint1`/`AGV_Joint2`.

### Why drop `joint_state_merger`

It exists to stitch two 7-joint messages into one 16-joint message. That made
sense when the waist was fake and both arms shared a fabricated stamp. It stops
making sense once each source has a real per-sample timestamp, because merging
forces a single stamp onto samples taken at different instants — destroying
exactly the information we are trying to preserve.

**Risk to check before deleting it:** anything downstream that assumes a single
`/joint_states` message contains all 16 joints. Grep for `/joint_states`
subscribers. `robot_state_publisher` does NOT handle partial messages (see the
note above); MoveIt's planning scene monitor does accumulate, but TF comes from
RSP, so partial messages break TF regardless of what MoveIt does. If something does depend on it, keep the merger
as an optional node behind a launch arg rather than in the default path.

### Why a separate waist node

- **One publisher per physical joint.** The waist is not part of either arm; it
  is a shared external axis that both controllers happen to report.
- **Verified feasible.** Two concurrent RDK sessions to the same controller work
  (§1). The waist node opens its own session to whichever arm is configured.
- **Negligible cost.** The controller pushes the UDP stream regardless of
  listeners — ~5 Mbps for 660-byte packets at 1 kHz, trivial on wired GbE. A
  third subscriber does not make the controller work harder.
- **Independent lifecycle.** Waist data stays available if an arm driver is down
  for maintenance.

**Trade-off accepted:** one extra process and one extra RDK session. If session
count ever becomes a constraint, the fallback is the waist node connecting to
whichever arm is least loaded, or folding the waist back into one arm driver
behind a `publish_waist` parameter.

---

## 4. Node specifications

### 4.1 Common acquisition pattern (all three nodes)

```
on_activate:
    start poll thread

poll thread (free-running, not rate-limited):
    loop:
        s = robot.states()
        if s.timestamp != last_timestamp:
            last_timestamp = s.timestamp
            slot.store(Sample(timestamp=s.timestamp, q=s.q, dq=s.dq, tau=s.tau))
        # no sleep — states() is a cached read, ~4 µs

publish timer @ publish_rate_hz:
    sample = slot.load()
    if sample is None or age(sample) > stale_threshold:
        warn (throttled); return
    msg.header.stamp = to_ros_time(sample.timestamp + offset_estimate)
    msg.position = sample.q[...]
    msg.velocity = sample.dq[...]
    msg.effort   = sample.tau[...]
    publish(msg)
```

Key properties:

- Acquisition rate is set by the device, not by us.
- `publish_rate_hz` becomes an honest **decimation factor** with a known
  relationship to the source, instead of an aperiodic sampler.
- The publish path never blocks on I/O.
- Single-slot latest-value semantics by default. A ring buffer is only needed if
  a consumer wants every sample; see §7.

**Concurrency note:** the slot must be written by the poll thread and read by the
timer callback without tearing. In Python, assigning a whole immutable `Sample`
object to an attribute is atomic under the GIL — no lock needed, provided you
replace the object rather than mutate it in place. If this is ever ported to C++,
use a double-buffer or `std::atomic<std::shared_ptr<Sample>>`.

### 4.2 `waist_driver` (new)

| | |
|---|---|
| Package | `flexiv_amr_driver` (or a new `aico2_waist_driver`) |
| Node name | `waist_driver` |
| RDK target | configurable `source_robot_sn`, default `Rizon4-063352` |
| Publishes | `/joint_states` — 2 joints only |
| Joint names | `AGV_Joint1` (yaw, `q[0]`), `AGV_Joint2` (pitch, `q[1]`) |

**Resolved 2026-09-18 — the names were misspelled (`Jiont`) and have been
fixed.** The original note said to pick one spelling and apply it everywhere
in a single commit; that happened. What made it safe: the typo was confined
to the two `<joint name=...>` strings in the URDF. The *links*
(`AGV_Yaw`, `AGV_Pitch`) and every mesh file were already spelled correctly,
and TF frames come from link names — so no TF frame ever carried the typo and
nothing in the transform tree changed. In-repo that left 5 functional
occurrences (URDF ×2, `waist_driver.yaml`, `waist_driver_node.py`'s
`_DEFAULT_JOINTS`, `joint_state_merger.py`'s `WAIST`), renamed together.
Anything outside this repo that looks these joints up **by name** — not by
frame — still needs checking.

Parameters:

```yaml
waist_driver:
  ros__parameters:
    source_robot_sn: "Rizon4-063352"
    publish_rate_hz: 200.0          # decimation from 1 kHz
    joint_names: ["AGV_Joint1", "AGV_Joint2"]
    use_device_timestamp: true
    offset_refresh_sec: 300.0
    stale_threshold_sec: 0.05
```

### 4.3 `arm_driver` (modify existing)

Changes only — the control path is untouched.

1. Replace the timer-driven `states()` read with the poll thread + slot pattern.
2. `header.stamp` from the device timestamp when `use_device_timestamp` is true;
   fall back to `now()` when false, so the old behaviour stays reachable.
3. Publish `q[2:9]` explicitly — make the 9-vs-7 offset a named constant with a
   comment, because it is not obvious from the RDK docs and will confuse the next
   reader.
4. Populate `velocity` from `dq[2:9]` and `effort` from `tau[2:9]`. Currently
   these are left empty; `dq` is a real measurement and is better than
   differentiating `q` downstream.
5. Add `publish_rate_hz` documentation making clear it is now decimation, and
   that 1000 Hz is available.
6. **Guard against double-publishing the waist.** The arm driver must publish
   only `q[2:9]`. If both an arm driver and the waist node publish the same joint
   name, `robot_state_publisher` takes whichever arrived last and the TF jitters
   between two sources.

### 4.4 Offset estimator (shared module)

Converts device timestamps to host time. Used by all three nodes.

```
estimate_offset(robot, n=200) -> (offset_sec, spread_sec):
    # Poll flat out and catch the instant the device timestamp increments.
    # This brackets the offset to the poll interval (~µs) rather than the
    # sample interval (~1 ms), which is what the earlier 291 µs figure was
    # actually measuring.
    samples = []
    prev_ts = None
    while len(samples) < n:
        t0 = time.monotonic()
        s = robot.states()
        t1 = time.monotonic()
        if prev_ts is not None and s.timestamp != prev_ts:
            # transition detected between t0 and t1
            samples.append(device_time(s.timestamp) - host_time_at((t0+t1)/2))
        prev_ts = s.timestamp
    return median(samples), iqr(samples)
```

Requirements:

- Re-estimate every `offset_refresh_sec` (default 300 s). The measured ~70 ppm
  drift is ~21 ms/hour — small per publish cycle, unacceptable over a session.
- Apply the offset as a **slew, not a step**, if it moves by more than a
  threshold. A step in `header.stamp` can push TF backwards in time and make
  tf2 discard the buffer.
- Publish the current offset and its spread on `~/clock_offset`
  (`diagnostic_msgs/DiagnosticStatus` or a small custom msg) so it is observable.
  A jump here is the first sign a controller rebooted.
- **Per-node, per-controller.** The two arms differ by 13 s. Do not share one
  estimate between them.

Estimated result: the earlier 291 µs figure should drop substantially, because
most of it was polling phase. This is unverified until implemented.

---

## 5. Parameters

| Parameter | Default | Notes |
|---|---|---|
| `publish_rate_hz` | 200.0 | Decimation from 1 kHz. 1000 is possible; see §7 on bandwidth |
| `use_device_timestamp` | true | false restores the old `now()` behaviour |
| `offset_refresh_sec` | 300.0 | Tracks the ~70 ppm drift |
| `offset_slew_limit` | 0.001 | Max step applied per refresh, seconds |
| `stale_threshold_sec` | 0.05 | Skip publish if newest sample is older than this |
| `publish_velocity` | true | From `dq`, not differentiated |
| `publish_effort` | true | From `tau` |
| `source_robot_sn` | — | Waist node only |

`publish_rate_hz` default of 200 is a deliberate middle ground: 4× better than
today, well under any bandwidth concern, and above the range where aliasing is
likely to matter. Raise it if a consumer needs finer resolution.

---

## 6. Verification

Each item is a measurement, not an assertion. Re-run the same analysis scripts
used for the baseline so the numbers are comparable.

**Acquisition**
- [ ] Poll thread achieves ~1000 unique samples/s per node
- [ ] `diff` of published `header.stamp` matches `publish_rate_hz` with tighter
      jitter than the current 0.17 ms
- [ ] Static noise on `/joint_states` matches the direct-RDK baseline
      (arms σ 0.89–1.61 µrad, waist σ 0.014/0.019 µrad). A mismatch means the
      pipeline is adding noise.

**Timestamps**
- [ ] Offset estimate converges and its spread is reported
- [ ] Offset tracks drift across a ≥1 h run without stepping
- [ ] `ros2 topic delay /joint_states` reflects real device-to-publish latency
- [ ] Left and right arms produce independent offsets ~13 s apart, as measured

**Waist**
- [ ] `AGV_Joint1`/`AGV_Joint2` carry real values, not 0.0
- [ ] TF `base_link → AGV_Yaw → AGV_Pitch` moves when the torso moves
      *(blocked: torso currently locked)*
- [ ] Values match the other arm's `q[0]`/`q[1]` to within one LSB

**Integration**
- [ ] `robot_state_publisher` produces a complete TF tree from three partial
      publishers
- [ ] No joint published by more than one node
- [ ] Arm driver survives the waist node starting and stopping, and vice versa
- [ ] All three recover from a controller disconnect without a restart

**Regression**
- [ ] `use_device_timestamp: false` reproduces current behaviour
- [ ] Existing `/joint_states` consumers still work, or are identified and fixed

---

## 7. Decisions deferred, with the information needed to make them

**Bandwidth at high publish rates.** 1 kHz × 9 joints × `JointState` is roughly
0.5 MB/s. Fine on the robot; the cross-machine link to the dev laptop has already
shown DDS flakiness on `/rgbd_image` and `/odometry/filtered` at lower rates. If
1 kHz is wanted off-robot, measure first, and consider a separate
high-rate topic so the default `/joint_states` stays light.

**Ring buffer vs latest-slot.** Latest-slot is right for TF, which only ever
wants the newest pose. A consumer that needs every sample — say, a dynamics
identification run — needs a buffer and a batch message. No such consumer exists
yet. Do not build it speculatively.

**PTP.** Host side is ready: hardware tx/rx timestamping, PHC0 live on
`enP2p1s0`. No PTP traffic on the segment, so nothing is configured today. The
open question is vendor-side and should be put to Flexiv precisely: *can the
Rizon controller participate in IEEE 1588 as master or slave, and is there a
configuration for it?* If yes, the offset estimator becomes a fallback rather
than the primary mechanism. Until then it is the primary mechanism.

**Joint name typo — DECIDED, fixed 2026-09-18.** Renamed to
`AGV_Joint1`/`AGV_Joint2` across the URDF, waist driver and merger in one
commit. Cheaper than this section assumed: links and meshes were already
correct, so TF frames were never affected. See §4.2.

---

## 8. Suggested implementation order

1. **Offset estimator as a standalone module + unit test.** Everything depends on
   it and it is the easiest piece to get subtly wrong.
2. **`waist_driver`.** Greenfield, no regression risk, and it delivers the
   highest-value missing data. Validates the whole pattern in isolation.
3. **Verify waist in TF**, with `joint_state_merger` still running and still
   publishing zeros — confirm the last-writer-wins behaviour and that the real
   values win, or temporarily rename to avoid the clash.
4. **Retrofit one arm driver.** Compare its output against the untouched second
   arm as a control.
5. **Retrofit the second arm driver.**
6. **Remove `joint_state_merger`** once all three publish independently and the
   consumer grep is clean.
7. **Re-run the full characterisation** through the new pipeline and compare
   against the baseline numbers in §1.

Steps 1–3 are independently useful and carry no risk to the working system. If
the work stalls after step 3, the waist data is still available and nothing has
regressed.
