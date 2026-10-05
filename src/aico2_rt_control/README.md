# aico2_rt_control

Real-time (1 kHz) joint control for the Rizon4 arms via the Flexiv RDK **C++**
API.

**Nothing here is built or tested yet**, and nothing here changes
`aico2_left_arm_driver`. That driver is Python, NRT, and works — it stays as it
is. This package exists so the RT route can be evaluated without putting the
working path at risk.

---

## Is Flexiv's example enough to write an RT controller through MoveIt?

Short answer: **it is enough to prove RT works, and not enough to be a MoveIt
controller.** The gap is ROS integration, real-time safety, and the host
kernel — not the RDK.

### What the examples give you

Support linked the RT *joint torque* example. The one that matters for
trajectory execution is **`intermediate1_realtime_joint_position_control.cpp`**,
because it streams positions rather than torques:

```cpp
robot.SwitchMode(rdk::Mode::RT_JOINT_POSITION);
...
robot.StreamJointPosition(target_pos, target_vel, target_acc);   // 1 ms task
```

Three things come for free, and they are the hard parts:

- **The RT loop itself.** `rdk::Scheduler::AddTask(fn, name, 1, max_priority())`
  plus `Start()`. You do not write timing code, `clock_nanosleep`, or thread
  priority handling.
- **No internal motion generator.** `StreamJointPosition` takes no `max_vel` /
  `max_acc` and does not re-plan. The controller tracks what you send. Every
  problem chased in `docs/open_issues.md` issues 1 and 1a — the trapezoid's
  unbounded jerk, the call-rate U-curve, the controller decelerating toward each
  setpoint — is structurally absent, because there is no setpoint-to-setpoint
  planning to interfere with.
- **The enable / fault / operational pattern**, identical to the Python driver.

The torque example also warns that its own impedance controller is "for demo
purpose only and has no performance guarantee", pointing at
`intermediate2_realtime_joint_impedance_control.cpp` for the robot's built-in
one. Worth reading both before choosing between `RT_JOINT_POSITION` (stiff) and
`RT_JOINT_IMPEDANCE` (compliant); the compliance work in
`docs/rdk_version_and_compliance.md` suggests impedance is the more useful mode
here.

### Problems in the torque example as shipped

`intermediate3_realtime_joint_torque_control.cpp` is worth reading and is not
worth copying. Four things are wrong with it on this robot:

1. **It reads past the end of its gain arrays.** `kImpedanceKp` and
   `kImpedanceKd` have seven entries, and the control loop runs
   `for (size_t i = 0; i < target_torque.size(); ++i)` where `target_torque` was
   sized `robot.info().DoF`. On a plain Rizon4 — the `Rizon4s-123456` in the
   example's own help text — DoF is 7 and it happens to work. On the AICO2 DoF
   is **9**: two external waist axes plus seven arm joints. Iterations 7 and 8
   index both gain arrays out of bounds, so the torque commanded to the waist
   axes is computed from whatever is in adjacent memory. This is worth reporting
   upstream; it affects any Rizon with external axes, not just this one.

2. **It replaces the robot's controller with a hand-written PD law** and says so:
   the comment calls it "for demo purpose only" with "no performance guarantee"
   and points at `intermediate2_realtime_joint_impedance_control.cpp` for the
   built-in one. For trajectory execution there is no reason to prefer the demo.

3. **The periodic task is not RT-safe.** It constructs two
   `std::vector<double>` per cycle, calls `robot.states()` twice per joint
   inside the loop rather than once per cycle, and can `throw` from inside the
   1 ms task. Allocation and exceptions in a 1 kHz loop are exactly what a
   missed deadline is made of.

4. **Smaller things.** `static unsigned int loop_counter` is function-local
   state that breaks if two arms ever run in one process; `ExecutePlan("PLAN-Home")`
   needs a plan of that name authored on the robot; and the "trajectory" is a
   hardcoded sine with no external input, so none of the handoff problem is
   demonstrated.

### What MoveIt actually hands a controller

This decides which RT mode is the right target, so it is worth being precise.
A `trajectory_msgs/JointTrajectoryPoint` has five fields:

`positions`, `velocities`, `accelerations`, `effort`, `time_from_start`.

MoveIt's planning pipeline fills the first three and the timestamp.
`effort` is left empty — OMPL plans in configuration space and the time
parameterization adapters produce a kinematic profile, not a dynamic one.
Nothing in `aico2_moveit_config` changes that.

Two consequences:

- **A torque controller cannot be driven from MoveIt output directly.** There
  are no torques in the message to forward. What you *can* do is write a
  control law that turns the kinematic triple into torque — a PD term on
  position and velocity error plus an inverse-dynamics feedforward on
  `accelerations`. That is computed-torque control, it is a real option, and it
  is precisely the thing `intermediate3` sketches badly: the gains are
  hardcoded constants with no feedforward term at all. Doing it properly needs
  the robot's dynamics (the RDK's `Model` class, per its dynamics example) and
  gain tuning on hardware.
- **`RT_JOINT_POSITION` is a field-for-field match.**
  `StreamJointPosition(pos, vel, acc)` takes the same three vectors the
  trajectory point carries, so the controller body is a lookup into the
  trajectory at the current time and one call. `RT_JOINT_IMPEDANCE` takes
  positions too and adds compliance using Flexiv's own tuned controller — the
  thing the torque demo warns it is not.

So: torque is reachable but means authoring and tuning a controller that the
robot already contains a better version of. Impedance or position streaming is
the route that uses what MoveIt produces as-is.


### What you still have to write

1. **ROS 2 integration.** The examples are standalone `main()` functions. A
   MoveIt controller needs a `FollowJointTrajectory` action server,
   `/joint_states`, and lifecycle management — all of which exist in Python
   today and would have to be written again in C++.

2. **Real-time safety, which the examples do not demonstrate.** The periodic
   task in `intermediate1` constructs three `std::vector<double>` per cycle.
   That is fine in a demo and wrong in a control loop: `malloc` can block.
   Nothing in a 1 kHz task may allocate, take a lock a non-RT thread holds, log,
   or throw. `src/rt_hold_probe.cpp` is written to that standard and the
   difference from the example is visible.

3. **Trajectory handoff.** MoveIt delivers a whole trajectory on a ROS executor
   thread; the 1 kHz task has to sample it without blocking on that thread. That
   needs a lock-free handoff — double buffer with an atomic swap, or an SPSC
   queue. This is the real design work and the examples say nothing about it.

4. **The external axes — answered, from the header.** `StreamJointPosition`
   "throw[s] `std::invalid_argument` if size of any input vector does not match
   robot DoF", so it takes the full 9 exactly as `SendJointPosition` does. The
   split comes from `robot.info()`, which the Python driver already reads: `DoF`
   9, `DoF_m` 7 for the arm, `DoF_e` 2 for the external waist axes, external
   first.

   `LockExternalAxes` is answered too: applicable control mode `IDLE`, taking
   effect "during primitive execution, **direct joint control**, and direct
   Cartesian control modes" (`robot.hpp:272`). Direct joint control covers the
   RT joint modes as much as the NRT ones, so the existing order — call it while
   IDLE, before `SwitchMode` — carries over unchanged. Nothing to ask support
   about on either count.


5. **The host.** This is the real risk. `scheduler.max_priority()` requests
   SCHED_FIFO at a high priority; on a kernel **without PREEMPT_RT** that is
   best-effort. The Jetson also runs Nav2, RTAB-Map, two camera pipelines and a
   DDS domain. In NRT a late command is covered by the robot's motion
   generator; in RT there is nothing covering for you, so a missed deadline is
   worse than the problem being solved. Measure before building.

### What we have, and what has to be asked for

Three separate things are needed, and they fail for different reasons. Checked
against the public v1.9 headers (`flexivrobotics/flexiv_rdk`, tag `v1.9`):

| | State | How it is obtained |
|---|---|---|
| The RT API | **Exists, public** | `include/flexiv/rdk/robot.hpp:501` declares `StreamJointPosition(positions, velocities, accelerations)`, applicable modes `RT_JOINT_IMPEDANCE, RT_JOINT_POSITION`. `mode.hpp` lists `RT_JOINT_TORQUE`, `RT_JOINT_IMPEDANCE`, `RT_JOINT_POSITION`, `RT_CARTESIAN_MOTION_FORCE`. Nothing is hidden. |
| The C++ library | **Not installed** | A download, free. See below. The Python wheel will never do: upstream names every file in `example_py/` `non_realtime_*` and every RT example is C++. The Jetson reporting `RT modes: []` and `Stream methods: []` is the correct current state of the bindings, not a stale wheel. |
| A licence permitting the RT modes | **Unknown — the actual gate** | `SwitchMode` "throw[s] `std::invalid_argument` if the requested mode is invalid **or unlicensed**" (`robot.hpp:205`), and `Robot`'s constructor throws "if the connected robot lacks a valid RDK license" (`robot.hpp:41`). This is the thing to ask Flexiv for, if we turn out not to have it. |

The licence is checkable without asking anyone. `RobotInfo` carries a
`license_type` field (`data.hpp:124`), in the same struct the driver already
reads `DoF` / `DoF_m` / `DoF_e` from:

```bash
python3 src/aico2_rt_control/scripts/check_rt_license.py Rizon4-063352
python3 src/aico2_rt_control/scripts/check_rt_license.py Rizon4R-062077
```

Read-only — it never enables the robot or switches mode. It does take the one
RDK session, so stop the Python driver first. Run it on both arms; the licence
is per robot, and they may differ.

### A detail from the header that explains the NRT behaviour

Worth recording, because it is the documented mechanism behind
`docs/open_issues.md` issue 1a. `SendJointPosition`'s own warning:

> Calling this function a second time while the motion from the previous call is
> still ongoing will trigger an online re-planning of the joint trajectory, such
> that the previous command is aborted and the new command starts to execute.

So streaming at 50 Hz means aborting and re-planning every 20 ms, by design.
Also note the NRT meaning of `velocities`: "Each joint will maintain this amount
of velocity when it reaches the target position" — a terminal condition for the
generator's plan, not feedforward. In RT the same argument *is* feedforward, and
`StreamJointPosition` carries no `max_vel` / `max_acc` because there is no plan
to bound. That difference is the whole reason for this package.


### Why RT is the target here

The decision is RT. The reason is not raw rate, it is ownership of the
interpolant.

In NRT the driver streams `SendJointPosition(q, dq, max_vel, max_acc)` and the
robot's internal motion generator re-plans toward each setpoint under those
caps. The motion that comes out is therefore a property of a generator that is
not ours, driven at a rate that was found empirically. That is the whole of
`docs/open_issues.md` issue 1a: 2 Hz is badly jerky because the generator
decelerates toward each setpoint, 50 Hz is smoothest, 190 Hz degrades again
because the robot is being pre-empted. Nothing in that curve is controllable
from here, and no value of the send rate makes it ours.

`StreamJointPosition(pos, vel, acc)` in `RT_JOINT_POSITION` takes no caps and
does no planning. The controller tracks what it is given. So the interpolation
becomes ours, it is deterministic — the same trajectory produces the same joint
path every run — and the velocity and acceleration from MoveIt go to the
controller as feedforward instead of as hints to a generator. The rate U-curve
stops existing, because there is no setpoint-to-setpoint planning left to
interfere with.

The cost is that nothing covers for a missed deadline. That is what
`rt_hold_probe` measures, and it is the one thing worth knowing before the arm
is driven this way.

### Design: trajectory buffer plus a 1 kHz sampler

MoveIt delivers a whole trajectory, sparsely — 22 points for a typical move.
The 1 kHz task needs 1000 setpoints a second. So the trajectory is buffered once
and sampled continuously; that resampling is the controller.

**Buffering.** Preallocate the buffers at startup and never allocate again:

```cpp
struct Waypoint {
    double t;                      // time_from_start, seconds
    double q[kMaxDof], dq[kMaxDof], ddq[kMaxDof];
};
struct TrajBuffer { Waypoint pts[kMaxPoints]; std::size_t n; };

TrajBuffer                     slots[2];
std::atomic<TrajBuffer*>       active{nullptr};   // nullptr = hold position
std::atomic<std::uint64_t>     consumed{0};       // bumped by the RT task
```

`kMaxPoints` 4096 and `kMaxDof` 9 is about 1.8 MB for both slots, which is
nothing, and makes an over-long trajectory a rejected goal rather than a
`malloc` in the control loop.

**Handoff.** One writer (the ROS executor thread), one reader (the RT task).
The writer fills the inactive slot, then `active.store(ptr, release)`; the
reader does `active.load(acquire)` once per cycle. Two slots are only safe if
the writer never overwrites a slot the reader could still be holding, so the RT
task bumps `consumed` after each swap it observes, and the writer waits for that
acknowledgement before reusing the old slot. **The writer blocks, never the
reader** — a few milliseconds on the ROS thread is free, and a new goal arrives
every few seconds against a loop running every millisecond.

**Sampling.** Per cycle, with `t` from `steady_clock` since the trajectory
started:

- Advance a cached segment index forward to the span with
  `pts[i].t <= t < pts[i+1].t`. `t` is monotonic, so this is O(1) amortized and
  needs no search.
- Evaluate a **quintic Hermite** on that segment from `(q, dq, ddq)` at both
  ends. It reproduces MoveIt's values exactly at the knots and is C² inside, so
  acceleration is continuous across waypoint boundaries. That matters: the
  discontinuous acceleration at a trapezoid's corners is precisely what made
  execution jerky before Ruckig, and a cubic interpolant here would reintroduce
  it at every knot.
- Past the last waypoint, hold its `q` with `dq = ddq = 0` and set an atomic
  done flag for the ROS thread to finish the action on.
- With `active == nullptr`, hold the `q` captured at mode entry — what
  `rt_hold_probe` already does.

**What must not be in the task:** allocation, locks a non-RT thread can hold,
logging, `throw`. Statistics accumulate in plain members the RT thread alone
writes, read after `Stop()`.

**What must be on the ROS thread instead:** the joint name-to-index map. MoveIt
sends URDF joint names; the RDK vector is external axes first
(`DoF_e` 2, then `DoF_m` 7). Resolve that when the goal arrives.

**Limit clamping is now mandatory.** In NRT the generator's `max_vel` /
`max_acc` were a backstop between a bad buffer and the joints. In RT there is
nothing there. Validate the trajectory against the URDF limits when the goal is
accepted, and check `robot.fault()` every cycle.

### Recommended order

1. Install the C++ RDK (below) and run `rt_hold_probe`. It answers point 4,
   point 5, and whether RT works on this arm at all, for the cost of one small
   program. If the loop cannot hold on this host, that is worth knowing before
   any of the above gets written.
2. Then the sampler and the action server, per the design above.
3. Keep the Python NRT driver working alongside it. It is the fallback if the
   host timing turns out not to hold, and the comparison that says whether RT
   actually improved the motion.


---

## Installing the C++ RDK

**The `flexivrdk` Python wheel does not satisfy this.** Verified on the robot:
the wheel's bindings register no RT modes and no `Stream*` methods. RT lives in
the C++ library, which is a separate artifact.

```bash
git clone -b v1.9 https://github.com/flexivrobotics/flexiv_rdk.git
cd flexiv_rdk
```

**The branch matters.** `v1.x` through `v1.9.3` is Rizon; `v2.x` is Enlight, and
its API is not source-compatible — on `main` (v2.1) `StreamJointPosition` takes
`const std::map<JointGroup, RtJointPositionCmd>&` instead of three vectors.
Cloning the default branch gives code that will not compile against anything
written for these arms.

Its dependencies are vendored by a helper script, then the library itself is a
prebuilt static archive downloaded at configure time from the GitHub release
(`libflexiv_rdk.aarch64-linux-gnu.ros2-jazzy.a` for this machine, with a SHA256
check). Follow that repo's README for the current invocation — it changes
between point releases, so it is deliberately not copied here. The one flag that
matters:

```
-DRDK_SUPPORT_ROS2_JAZZY=ON
```

Without it you get `libflexiv_rdk.aarch64-linux-gnu.a`, which statically links
Fast-RTPS and Fast-CDR and will collide with ROS 2's copies as soon as anything
links `rclcpp`.

Install it where CMake can find it, then:

```bash
cd ~/flexiv_ws
colcon build --packages-select aico2_rt_control
```

Until `find_package(flexiv_rdk)` succeeds this package builds nothing and emits
a warning rather than failing, so it cannot break a workspace build.

---

## rt_hold_probe

Holds every joint where it is for a few seconds and reports what the 1 kHz loop
actually achieved. The arm should not travel.

```bash
ros2 run aico2_rt_control rt_hold_probe Rizon4-063352
ros2 run aico2_rt_control rt_hold_probe Rizon4-063352 --seconds 20
```

**Stop the Python arm driver first.** The RDK allows one session per robot, so
`full_system.launch.py` or `arms.launch.py` must not be running. E-stop
released, motion bar in Auto (Remote), as always.

Output:

```
cycles          4998
expected        5000
mean period     1.0004 ms  (nominal 1.000)
min period      0.9120 ms
max period      1.3400 ms
missed >1.5ms   0
```

How to read it:

| Result | Meaning |
|---|---|
| `SwitchMode` throws `std::invalid_argument` | the mode is invalid **or unlicensed** (`robot.hpp:205`). Check `license_type` with `scripts/check_rt_license.py`; if RT is not licensed, that is the request to make to Flexiv |
| misses 0, max near 1 ms | the host can hold the loop; RT is worth pursuing |
| misses over 1% of cycles | not safe to drive the arm with on this host. Either isolate a core (`isolcpus`, `taskset`), move the arms off the Jetson, or stay on NRT |
| fault during the loop | note what the robot reports; the probe only holds position, so a fault means the stream itself was rejected |

Run it twice: once with the rest of the stack stopped, once with Nav2 and the
cameras running. The difference between those two numbers is the thing that
decides whether RT is viable here, and no amount of reading the examples
answers it.
