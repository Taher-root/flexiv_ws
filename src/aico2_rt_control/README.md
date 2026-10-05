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

4. **The external axes.** Most of this is already known and does not need
   asking Flexiv. `robot.info()` reports the split directly, and the Python
   driver reads all three fields at startup (`_init_rdk_dof_from_robot`): `DoF`
   9, `DoF_m` 7 for the arm, `DoF_e` 2 for the external waist axes, with the
   external axes first in the vector. In NRT this is proven on hardware —
   `SendJointPosition` takes the full 9 and the waist moves under
   `control_waist:=true`, after `LockExternalAxes(False)` in IDLE before
   `SwitchMode`.

   What is genuinely unknown is only the RT side: whether `StreamJointPosition`
   expects the same 9, and whether `LockExternalAxes` behaves the same way with
   an RT mode active. Neither is checkable from Python — the wheel registers no
   RT modes — but both are answered by reading `flexiv/rdk/robot.hpp` once the
   C++ RDK is installed, and confirmed by one run of the probe.

5. **The host.** This is the real risk. `scheduler.max_priority()` requests
   SCHED_FIFO at a high priority; on a kernel **without PREEMPT_RT** that is
   best-effort. The Jetson also runs Nav2, RTAB-Map, two camera pipelines and a
   DDS domain. In NRT a late command is covered by the robot's motion
   generator; in RT there is nothing covering for you, so a missed deadline is
   worse than the problem being solved. Measure before building.

### Recommended order

1. Install the C++ RDK (below) and run `rt_hold_probe`. It answers points 4 and
   5 and whether RT works on this arm at all, for the cost of one small program.
2. Only if the timing holds, write the ROS controller.
3. Keep the Python NRT driver as the default either way. It works, and RT is
   the wrong tool for executing a pre-planned trajectory that the robot can
   already interpolate well.

---

## Installing the C++ RDK

**The `flexivrdk` Python wheel does not satisfy this.** Verified on the robot:
the wheel's bindings register no RT modes and no `Stream*` methods. RT lives in
the C++ library, which is a separate artifact.

```bash
git clone -b v1.9 https://github.com/flexivrobotics/flexiv_rdk.git
cd flexiv_rdk
```

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
| `SwitchMode` throws | RT is not available on this arm as installed — the answer, cheaply |
| misses 0, max near 1 ms | the host can hold the loop; RT is worth pursuing |
| misses over 1% of cycles | not safe to drive the arm with on this host. Either isolate a core (`isolcpus`, `taskset`), move the arms off the Jetson, or stay on NRT |
| fault during the loop | note what the robot reports; the probe only holds position, so a fault means the stream itself was rejected |

Run it twice: once with the rest of the stack stopped, once with Nav2 and the
cameras running. The difference between those two numbers is the thing that
decides whether RT is viable here, and no amount of reading the examples
answers it.
