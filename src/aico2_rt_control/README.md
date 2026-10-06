# aico2_rt_control

Real-time (1 kHz) joint control for the Rizon4 arms via the Flexiv RDK **C++**
API.

**Nothing here is built or tested yet**, and nothing here changes
`aico2_left_arm_driver`. That driver is Python, NRT, and works — it stays as it
is, as the fallback and as the comparison.

Both arms already carry `RDK-Professional`, so RT is licensed and the API is
public. The one remaining blocker is installing the C++ library, which is a free
download. See [the three prerequisites](#the-three-prerequisites-and-where-each-one-stands).

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

   Flexiv suggest a lighter alternative worth keeping in mind: keep planning in
   Python and hand the trajectory to a small C++ program that only streams it,
   over a file, shared memory or a socket. That avoids rewriting the action
   server and the state publishing, at the cost of a process boundary on the
   path a goal travels. It is a reasonable first integration if the probe looks
   good but the full C++ node is not worth it yet — and the sampler design
   below is the same either way, since the handoff it describes is already
   between a non-RT producer and the RT task.

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


5. **The host. This is the real risk, and Flexiv make it a requirement.**
   Their three RT prerequisites are C++, an RDK Professional licence, and "a
   real-time capable Linux PC (see the RDK manual's real-time kernel setup)
   with a wired connection to the robot". The first two we have; the third we
   do not, as things stand. `scheduler.max_priority()` requests SCHED_FIFO at a
   high priority, which on a kernel without PREEMPT_RT is best-effort, and the
   Jetson also runs Nav2, RTAB-Map, two camera pipelines and a DDS domain. In
   NRT a late command is covered by the robot's motion generator; in RT nothing
   covers for you, so a missed deadline is worse than the problem being solved.

   Check it before installing anything — it is free:

   ```bash
   bash src/aico2_rt_control/scripts/check_rt_host.sh
   ```

   It reports the preemption flavour, `/sys/kernel/realtime`, `isolcpus`, the
   CPU governor and `nvpmodel` state, the RT scheduling limits, and whether the
   route to each arm is wired. Then measure with `rt_hold_probe` rather than
   arguing from the kernel name: if misses turn out to be load-dependent,
   isolating a core may be enough, and if they persist on an idle host the arms
   want an RT kernel or a different machine.

### The three prerequisites, and where each one stands

Checked against the public v1.9 headers (`flexivrobotics/flexiv_rdk`, tag
`v1.9`) and against both arms. Nothing here needs asking Flexiv.

| | State | Evidence |
|---|---|---|
| The RT API | **Public, exists** | `robot.hpp:501` declares `StreamJointPosition(positions, velocities, accelerations)`, applicable modes `RT_JOINT_IMPEDANCE, RT_JOINT_POSITION`. `mode.hpp` lists `RT_JOINT_TORQUE`, `RT_JOINT_IMPEDANCE`, `RT_JOINT_POSITION`, `RT_CARTESIAN_MOTION_FORCE`. Nothing hidden or gated. |
| A licence permitting RT | **Already held, both arms** | `license_type` reads `RDK-Professional+TDK-Standard` on Rizon4-063352 and Rizon4R-062077, and the RDK logs `Validated license: RDK-Professional` on connect. Robot software v3.11, RDK v1.9. |
| The C++ library | **Not installed — the only remaining blocker** | A free download, and the one thing left to do. See [Installing the C++ RDK](#installing-the-c-rdk). |

So the gate that looked like a licence request is not one. `SwitchMode` throws
`std::invalid_argument` when a mode is "invalid or unlicensed"
(`robot.hpp:205`), and `Robot`'s constructor throws if the robot "lacks a valid
RDK license" (`robot.hpp:41`) — neither applies here.

To re-check at any time, per arm (the licence is per robot):

```bash
python3 src/aico2_rt_control/scripts/check_rt_license.py Rizon4-063352
python3 src/aico2_rt_control/scripts/check_rt_license.py Rizon4R-062077
```

Read-only — it never enables the robot or switches mode. It does take the one
RDK session, so stop the Python driver first.


### Why the NRT path behaves as it does

Worth recording, because it is the documented mechanism behind
`docs/open_issues.md` issue 1a. `SendJointPosition`'s own warning:

> Calling this function a second time while the motion from the previous call is
> still ongoing will trigger an online re-planning of the joint trajectory, such
> that the previous command is aborted and the new command starts to execute.

Flexiv's engineering team put it more precisely (2026-10-05): the call does not
create a trajectory segment at all. **Each call replaces the current target**,
and a 1 kHz generator on the robot chases the most recent one as fast as
`max_vel` / `max_acc` allow, with no knowledge of when the next command is due
and **no jerk limit** — so acceleration switches abruptly between `+max_acc`, 0
and `-max_acc`. See `docs/open_issues.md` issue 1a for the full account.
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

### Two constraints Flexiv state explicitly for RT

From their guidance of 2026-10-05, and both are goal-acceptance checks rather
than things the RT task can fix:

1. **The trajectory must start at the robot's current position with zero
   velocity.** MoveIt plans from its own idea of the start state, which comes
   from `/joint_states` and so is a sample up to one publish period old. The
   first point will therefore not match `states().q` exactly. Decide this at
   goal acceptance: reject if the gap exceeds a tolerance, and otherwise close
   it with a short generated lead-in — not by streaming the mismatch, because
   nothing smooths it. Likewise a non-zero initial velocity in the plan is a
   reject, not something to feed forward.
2. **The command stream must be continuous, because the robot does not smooth
   RT commands.** Three practical consequences. A new goal arriving mid-motion
   cannot simply swap the buffer — the swap has to happen at a point where
   position *and* velocity are continuous, or be preceded by a braking ramp.
   Finishing a trajectory means holding its last point, not dropping to
   whatever the next source says. And a cycle that fails to produce a setpoint
   is not a no-op: it is a discontinuity, which is the whole reason the host
   timing has to be measured first.

### Recommended order

1. Run `scripts/check_rt_host.sh`. Costs nothing, needs no RDK, and answers
   whether the host meets Flexiv's stated requirement before any time goes into
   the install.
2. Install the C++ RDK (below) and run `rt_hold_probe`, twice: stack stopped,
   then with Nav2 and the cameras running. That is the measurement that decides
   this, and it is one small program.
3. Then the sampler and the action server, per the design above.
4. Keep the Python NRT driver working alongside it. It is the fallback if the
   host timing turns out not to hold, and the comparison that says whether RT
   actually improved the motion.


---

## Installing the C++ RDK

This is the only thing standing between here and RT. Both arms are licensed
(see above) and the API is public; the library is a free download.

**The `flexivrdk` Python wheel does not satisfy this.** Verified on the robot:
the wheel's bindings register no RT modes and no `Stream*` methods. That is
upstream's design, not a stale install — every file in the RDK's `example_py/`
is named `non_realtime_*` and every RT example is C++.

```bash
git clone -b v1.9 https://github.com/flexivrobotics/flexiv_rdk.git
```

**The branch matters.** `v1.x` through `v1.9.3` is Rizon; `v2.x` is Enlight, and
its API is not source-compatible — on `main` (v2.1) `StreamJointPosition` takes
`const std::map<JointGroup, RtJointPositionCmd>&` instead of three vectors.
Cloning the default branch gives code that will not compile against anything
written for these arms.

Dependencies first — Eigen3, spdlog, Fast-RTPS, Fast-CDR, RBDyn — vendored into
a prefix of your choosing by the RDK's own script:

```bash
sudo apt install build-essential cmake
cd flexiv_rdk/thirdparty
bash build_and_install_dependencies.sh ~/rdk_install
```

Then the library itself. It is a prebuilt static archive the CMake project
downloads at configure time from the GitHub release, with a SHA256 check against
`lib/*.sha256`:

```bash
cd flexiv_rdk
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/rdk_install -DRDK_SUPPORT_ROS2_JAZZY=ON
cmake --build . --target install --config Release
```

`-DRDK_SUPPORT_ROS2_JAZZY=ON` is not optional here. It selects
`libflexiv_rdk.aarch64-linux-gnu.ros2-jazzy.a` instead of
`libflexiv_rdk.aarch64-linux-gnu.a`; the plain archive statically links Fast-RTPS
and Fast-CDR, and so does ROS 2, so the two collide as soon as anything links
`rclcpp`. Flexiv ships the variant precisely so they can coexist. Without the
flag, `CMakeLists.txt` falls through to the plain archive silently.

Then build this package, pointing CMake at the prefix:

```bash
cd ~/flexiv_ws
colcon build --packages-select aico2_rt_control \
    --cmake-args -DCMAKE_PREFIX_PATH=$HOME/rdk_install
```

Until `find_package(flexiv_rdk)` succeeds this package builds nothing and emits
a warning rather than failing, so it cannot break a workspace build. The warning
in a `colcon build` log is the expected state before this is done, not an error.

The dependencies install as shared libraries under the prefix, so a binary needs
to find them at runtime:

```bash
LD_LIBRARY_PATH=$HOME/rdk_install/lib ros2 run aico2_rt_control rt_hold_probe Rizon4-063352
```

If that becomes tiresome, set `BUILD_RPATH`/`INSTALL_RPATH` on the target in
`CMakeLists.txt` rather than exporting `LD_LIBRARY_PATH` globally — a
system-wide `LD_LIBRARY_PATH` pointing at a prefix that carries its own Fast-RTPS
is a good way to break unrelated ROS 2 nodes.

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
