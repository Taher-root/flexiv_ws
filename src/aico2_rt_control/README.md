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


5. **The host — measured, and it passes.** Flexiv's three RT prerequisites are
   C++, an RDK Professional licence, and "a real-time capable Linux PC (see the
   RDK manual's real-time kernel setup) with a wired connection to the robot".
   All three are satisfied. Measured on `qc-ubuntu` (2026-10-06) with
   `scripts/check_rt_host.sh`:

   | | Result |
   |---|---|
   | Kernel | `6.6.110 #1 SMP PREEMPT_RT`, `/sys/kernel/realtime` = 1, `CONFIG_PREEMPT_RT=y` |
   | Cores | 18, `rcu_nocbs=0-17` already on the kernel command line |
   | Governor | `performance` |
   | Link to both arms | `eth0`, wired, 1000 Mb/s |
   | Ping, 20 packets | 0.194 / 0.200 / 0.238 ms, mdev **0.009 ms** |
   | `isolcpus` | not set — the one remaining gap |
   | RT throttle | `sched_rt_runtime_us` 950000 |

   This is a better starting point than expected. A PREEMPT_RT kernel with
   `rcu_nocbs` across all cores and a performance governor is a deliberately
   RT-tuned configuration, not a default, and a 0.009 ms ping deviation over
   wired gigabit means the network is not going to be the jitter source.

   **Note this is not the Jetson.** 18 cores, x86_64, no `nvpmodel`. Earlier
   notes in this package assumed the arms would be driven from the Jetson
   alongside Nav2, RTAB-Map and the cameras; on this host that assumption needs
   re-checking, and if the RT loop and the perception stack live on different
   machines the contention risk largely goes away. Worth settling before
   interpreting `rt_hold_probe` under load.

   What is left is to measure, not to argue from the kernel name. Two things
   stay open until `rt_hold_probe` runs: whether the loop holds while the rest
   of the stack is running, and whether `isolcpus` or a `taskset` pin is needed.
   With PREEMPT_RT and a performance governor it may well not be, which is
   exactly why it is worth measuring before editing the boot configuration.

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
