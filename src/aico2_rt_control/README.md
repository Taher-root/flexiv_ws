# aico2_rt_control

Real-time (1 kHz) joint control for the Rizon4 arms via the Flexiv RDK **C++**
API.

**Nothing here is built or tested yet**, and nothing here changes
`aico2_left_arm_driver`. That driver is Python, NRT, and works — it stays as it
is, as the fallback and as the comparison.

Both arms carry `RDK-Professional`, the API is public, and the host runs a
PREEMPT_RT kernel on wired gigabit — so every prerequisite Flexiv state is met
except one. The last blocker is installing the C++ library, a free download. See [the three prerequisites](#the-three-prerequisites-and-where-each-one-stands).

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

### The three prerequisites, and where each one stands

Checked against the public v1.9 headers (`flexivrobotics/flexiv_rdk`, tag
`v1.9`), against both arms, and against the host. Nothing here needs asking
Flexiv, and only one item is outstanding.

| | State | Evidence |
|---|---|---|
| The RT API | **Public, exists** | `robot.hpp:501` declares `StreamJointPosition(positions, velocities, accelerations)`, applicable modes `RT_JOINT_IMPEDANCE, RT_JOINT_POSITION`. `mode.hpp` lists `RT_JOINT_TORQUE`, `RT_JOINT_IMPEDANCE`, `RT_JOINT_POSITION`, `RT_CARTESIAN_MOTION_FORCE`. Nothing hidden or gated. |
| A licence permitting RT | **Already held, both arms** | `license_type` reads `RDK-Professional+TDK-Standard` on Rizon4-063352 and Rizon4R-062077, and the RDK logs `Validated license: RDK-Professional` on connect. Robot software v3.11, RDK v1.9. |
| The C++ library | **Not installed — the only remaining blocker** | A free download, and the one thing left to do. See [Installing the C++ RDK](#installing-the-c-rdk). |
| A real-time capable host | **Met** | `qc-ubuntu` runs `6.6.110 PREEMPT_RT` with `rcu_nocbs=0-17` and a performance governor, wired gigabit to both arms at 0.2 ms with 0.009 ms deviation. Point 5 of [what you still have to write](#what-you-still-have-to-write) has the measurements. |

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

**Use the `v1.9` tag on this robot.** Not the newest 1.x, and not `main`.
Measured on hardware (2026-10-06):

| client | result against robot software v3.11 |
|---|---|
| `flexivrdk` Python wheel **v1.9** | connects, reads state, drives the arms — in production use here |
| C++ **v1.9.4.1** (the `v1.x` branch head) | connects, then `Version of this client is incompatible with robot [Rizon4-063352]` |

So the RDK client version is checked against the robot's software version, and
1.9.4.1 is too new for v3.11. `v1.9` matches the wheel that already works, which
makes it the safe choice. Nothing in either repo documents this matrix — it has
to be found by trying.

The cost of staying on v1.9 is a harder build, because the shared-library
rework landed in **v1.9.4**:

| versions | library | consequences |
|---|---|---|
| `v1.9` – `v1.9.3` | static archive, in plain and `ros2-jazzy` variants | needs `-DRDK_SUPPORT_ROS2_JAZZY=ON`, the ROS 2 dependency script, and spdlog + fmt linked by the consumer |
| `v1.9.4` onwards | one self-contained `.so`, internal symbols hidden | Eigen is the only dependency; no flag, one dependency script |

`main` is `v2.1`/`v2.2`, which is Enlight, and its `StreamJointPosition` takes
`const std::map<JointGroup, RtJointPositionCmd>&` rather than three vectors — so
it is wrong regardless of the version check.

The API this package uses is identical across all of them:
`StreamJointPosition(positions, velocities, accelerations)`, `RT_JOINT_POSITION`
and `RT_JOINT_IMPEDANCE` in `mode.hpp`, and `license_type` / `DoF_m` / `DoF_e`
in `RobotInfo`. So moving between versions is a build concern, not a code one.

### Building v1.9

```bash
sudo apt install build-essential cmake libspdlog-dev libfmt-dev
bash src/aico2_rt_control/scripts/install_rdk.sh
```

The script does the whole sequence and refuses to continue when something is
wrong, which is worth more than it sounds: the two things that must be right are
silent when they are not. It sources ROS 2 before the dependency script, uses
`build_and_install_dependencies_not_in_ros2.sh`, configures with
`-DRDK_SUPPORT_ROS2_JAZZY=ON`, then **checks which Fast-DDS and Fast-CDR CMake
actually selected** and stops if they resolved under the prefix rather than
under ROS 2. It also refuses to reuse a prefix that already contains its own
Fast-DDS, which is the state the wrong dependency script leaves behind and which
reconfiguring does not undo. `--force` removes the prefix and the clone and
starts over.

It is also one command, which matters: pasting the sequence line by line lets
`apt` consume the lines that follow it, so the clone and the build silently
never run.

The equivalent by hand, if preferred:

```bash
git clone -b v1.9 https://github.com/flexivrobotics/flexiv_rdk.git
source /opt/ros/jazzy/setup.bash

cd flexiv_rdk/thirdparty
bash build_and_install_dependencies_not_in_ros2.sh ~/rdk_install

cd .. && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/rdk_install -DRDK_SUPPORT_ROS2_JAZZY=ON
cmake --build . --target install --config Release
```

Two dependency scripts exist in `thirdparty/`, and the wrong one gives several
hundred undefined `eprosima::fastcdr::*` symbols rather than any clear message:

| script | installs | for |
|---|---|---|
| `build_and_install_dependencies.sh` | Eigen, spdlog, tinyxml2, yaml-cpp, foonathan_memory, **Fast-CDR v1.0.28**, **Fast-DDS v2.6.10**, Boost, SpaceVecAlg, RBDyn | the plain archive |
| `build_and_install_dependencies_not_in_ros2.sh` | **Boost, SpaceVecAlg, RBDyn only** | the `ros2-jazzy` archive — ROS 2 supplies the rest |

`-DRDK_SUPPORT_ROS2_JAZZY=ON` is not optional. It selects
`libflexiv_rdk.aarch64-linux-gnu.ros2-jazzy.a` instead of
`libflexiv_rdk.aarch64-linux-gnu.a`; the plain archive statically links
Fast-RTPS and Fast-CDR, as does ROS 2, so the two collide the moment anything
links `rclcpp`. `CMakeLists.txt:50` defaults it `OFF` and falls through to the
plain archive silently.

`libspdlog-dev` and `libfmt-dev` are needed because the static archive
references spdlog and fmt without carrying them. Ubuntu 24.04 ships spdlog
1.12.0 and fmt 9.1.0, and `fmt::v9` is exactly the namespace the archive's
undefined symbols name. This package's `CMakeLists.txt` finds both with
`find_package(... QUIET)` and links them when present, so the same workspace
builds against either RDK layout.

**Check the configure output before building:**

```
-- Found fastrtps v2.14.x: /opt/ros/jazzy/share/fastrtps/cmake     correct
-- Found fastcdr  v2.x:    /opt/ros/jazzy/lib/cmake/fastcdr        correct

-- Found fastrtps v2.6.10: /root/rdk_install/share/fastrtps/cmake  wrong script
-- Found fastcdr  v1.0.28: /root/rdk_install/lib/cmake/fastcdr     wrong script
```

### If the robot software is ever updated

`v1.9.4`+ is a much easier build — one `.so`, Eigen only, no ROS 2 flag, no
version to mismatch. It is the right target once the robot software supports it.
Worth asking Flexiv which RDK versions match robot software v3.11 and whether an
update is advisable, since neither is documented.

Then build this package, pointing CMake at the prefix:

```bash
cd ~/flexiv_ws
colcon build --packages-select aico2_rt_control \
    --cmake-args -DCMAKE_PREFIX_PATH=$HOME/rdk_install
```

Until `find_package(flexiv_rdk)` succeeds this package builds nothing and emits
a warning rather than failing, so it cannot break a workspace build. The warning
in a `colcon build` log is the expected state before this is done, not an error.

The RDK is a shared library in its own prefix, which is not on the default
loader path. `CMakeLists.txt` sets `CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE` so
that path is baked into the binary and nothing extra is needed at runtime:

```bash
ros2 run aico2_rt_control rt_hold_probe Rizon4-063352
```

**Do not set `LD_LIBRARY_PATH` to the prefix.** Assigning it rather than
appending drops ROS 2's own `lib` directory, and `ros2` then fails with
`ImportError: librcl_action.so: cannot open shared object file`. If you ever do
need it, append: `LD_LIBRARY_PATH=$HOME/rdk_install/lib:$LD_LIBRARY_PATH`.

---

## Troubleshooting the build

**`Version of this client is incompatible with robot [Rizon4-...]`**, after
connecting successfully.

The RDK client is newer than the robot's software. These arms run v3.11, which
accepts `v1.9` and rejects `v1.9.4.1`. Rebuild from the `v1.9` tag — see
[Installing the C++ RDK](#installing-the-c-rdk). The prefix must be rebuilt, not
just reconfigured.

**Hundreds of `undefined reference to eprosima::fastcdr::Cdr::...`** such as
`CdrSizeCalculator`, `CdrVersion`, `Cdr::set_encoding_flag`,
`PortBasedTransportDescriptor`, coming from `libflexiv_rdk.a`.

Those exist only in Fast-CDR 2.x / Fast-DDS 2.14.x, which is what the
`ros2-jazzy` archive expects. `build_and_install_dependencies.sh` installs
Fast-CDR 1.0.28 and Fast-DDS 2.6.10 into the prefix, where they shadow ROS 2's
copies. Use `build_and_install_dependencies_not_in_ros2.sh` with ROS 2 sourced,
and rebuild the prefix from scratch:

```bash
rm -rf ~/rdk_install ~/flexiv_rdk
git clone -b v1.9 https://github.com/flexivrobotics/flexiv_rdk.git
source /opt/ros/jazzy/setup.bash
cd flexiv_rdk/thirdparty
bash build_and_install_dependencies_not_in_ros2.sh ~/rdk_install
cd .. && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=~/rdk_install -DRDK_SUPPORT_ROS2_JAZZY=ON
cmake --build . --target install --config Release
```

**`undefined reference to spdlog::details::log_msg::log_msg` or `fmt::v9::*`
coming from `libflexiv_rdk.a`.**

The v1.9 static archive references spdlog and fmt without carrying them.
`sudo apt install libspdlog-dev libfmt-dev` and reconfigure; this package links
both when `find_package` finds them.

**`undefined reference to fmt::v9::...` in `rt_hold_probe.cpp.o` itself** (not
in the RDK library).

Something in this package included `<spdlog/spdlog.h>`. Flexiv's examples log
with spdlog, but the RDK headers need only Eigen, and from v1.9.4.1 the library
hides its internal symbols — so including spdlog pulls in Ubuntu's build of it,
which links fmt externally, and leaves the translation unit needing `fmt`. The
fix taken here was to drop spdlog and use `iostream`; linking `fmt` would also
work but adds a dependency for nothing.

**`ImportError: librcl_action.so: cannot open shared object file`** when running
`ros2`.

`LD_LIBRARY_PATH` was *assigned* instead of appended, so ROS 2's `lib`
directory fell off it. Nothing is wrong with the install. The RPATH makes the
variable unnecessary — just drop it.

**`No executable found` from `ros2 run`, and the build logged
`flexiv_rdk (C++) not found -- skipping RT targets`.**

The RDK is not installed on *this* machine, for *this* user. The warning is the
package working as designed: `find_package(flexiv_rdk QUIET)` missed, so the
target was never created and nothing was installed for `ros2 run` to find.

**The prefix is per host and per user.** Installing it as `root` on one machine
does nothing for another user or another box — `$HOME/rdk_install` resolves
differently, and `/root/rdk_install` is not readable as a normal user anyway.
Each machine that needs to run RT needs its own install.

The warning prints the hostname, `CMAKE_PREFIX_PATH` and `HOME` it searched, so
compare those against where the install actually went.

**Also note CMake caches the result.** Once the prefix exists, a plain rebuild
may not re-run configure — it will finish in a fraction of a second and still
build nothing. Force it:

```bash
colcon build --packages-select aico2_rt_control \
    --cmake-force-configure --cmake-args -DCMAKE_PREFIX_PATH=$HOME/rdk_install
```

or `rm -rf build/aico2_rt_control install/aico2_rt_control` first.

**`*** stack smashing detected ***: terminated`** shortly after the RDK banner.

**Confirmed on hardware (2026-10-06) to be the library, not this workspace.**
Flexiv's own `basics1_display_robot_states`, built from their unmodified
`example/CMakeLists.txt` against the same prefix, aborts identically:

```
[info] | | | | | |  Flexiv RDK v1.9  | | | | | |
*** stack smashing detected ***: terminated
Aborted (core dumped)
```

The combination is the v1.9 **`ros2-jazzy`** prebuilt archive linked against the
Fast-DDS and Fast-CDR that ROS 2 Jazzy currently ships. From the example's own
link line:

```
/root/rdk_install/lib/libflexiv_rdk.a
/opt/ros/jazzy/lib/libfastrtps.so.2.14.6
/opt/ros/jazzy/lib/libfastcdr.so.2.2.7
```

Nothing states which versions that archive was built against, and a struct
layout change in a patch release would present exactly this way: it links
cleanly, then overruns a stack buffer inside the `Robot` constructor. Worth
reporting — it is a two-command reproducer using only Flexiv's own example and
stock Jazzy.

Note the apparent crash point is not the real one. `abort()` discards buffered
stdout, so the last line shown is simply the last one that flushed. Run the
binary directly rather than through `ros2 run`, and under `stdbuf -o0 -e0`, to
see how far it really gets.

**The way round it:** the plain (non-ROS 2) archive with Flexiv's own vendored
dependency versions, in a separate prefix, built with ROS 2 *not* sourced:

```bash
bash src/aico2_rt_control/scripts/install_rdk.sh --standalone

# Everything after this must run with ROS 2 out of the environment:
bash src/aico2_rt_control/scripts/noros.sh

# ...and inside that shell, check Flexiv's own example before anything of ours
cd ~/flexiv_rdk_standalone/example
cmake -S . -B build -DCMAKE_PREFIX_PATH=$HOME/rdk_standalone
cmake --build build -j
./build/basics1_display_robot_states Rizon4-063352

# Flexiv's examples need this: see the note below
export LD_LIBRARY_PATH=$HOME/rdk_standalone/lib
./build/basics1_display_robot_states Rizon4-063352

# if that prints robot states, the archive works; then
cd ~/flexiv_ws/src/aico2_rt_control/standalone
cmake -S . -B build -DCMAKE_PREFIX_PATH=$HOME/rdk_standalone
cmake --build build -j
./build/rt_hold_probe Rizon4-063352
```

**Flexiv's examples need `LD_LIBRARY_PATH` set to the prefix's `lib`**, and
`rt_hold_probe` does not. The reason is `DT_RUNPATH` versus `DT_RPATH`. The RDK
links Fast-RTPS, which itself needs foonathan_memory; both are in the prefix.
Modern linkers emit `DT_RUNPATH`, and `ld.so` applies an object's `RUNPATH`
only to that object's *own* direct dependencies — so the executable's `RUNPATH`
finds `libfastrtps.so` but not the `libfoonathan_memory.so` that libfastrtps
requires, and startup fails with `cannot open shared object file`. `DT_RPATH`
*is* inherited down the chain. `standalone/CMakeLists.txt` therefore links with
`-Wl,--disable-new-dtags`; Flexiv's `example/CMakeLists.txt` does not, so set
the variable for theirs.

This does not contradict the warning elsewhere about `LD_LIBRARY_PATH`. That
warning is about *assigning* it in a shell that has ROS 2 sourced, which drops
ROS's own `lib` directory and breaks `ros2`. Here ROS 2 is deliberately absent,
so setting it is correct and affects nothing else.

`noros.sh` is the single definition of "ROS-free" here — it unsets
`CMAKE_PREFIX_PATH`, `AMENT_PREFIX_PATH`, `LD_LIBRARY_PATH`, `PYTHONPATH`,
`ROS_DISTRO` and the rest, and `install_rdk.sh --standalone` re-execs through it
rather than keeping its own copy of the list. Run it with no arguments for an
interactive shell, or with a command to scrub just that one. The *configure*
step is what matters: once a binary is linked against the prefix's Fast-DDS it
carries an RPATH and the sonames differ (2.6 vs 2.14), so running it later from
a normal shell is fine.

`--standalone` re-execs itself with the ROS 2 variables removed
(`CMAKE_PREFIX_PATH`, `AMENT_PREFIX_PATH`, `LD_LIBRARY_PATH` and the rest) and
configures with `-DCMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH=OFF`. That is not
belt-and-braces: `find_package` consults the *environment* `CMAKE_PREFIX_PATH`
as well as the cache variable, so a shell with ROS 2 sourced silently links
Jazzy's Fast-DDS again — the exact thing this variant exists to avoid — and
scrubbing by hand is unreliable when a dotfile sources ROS. The script checks
afterwards that Fast-DDS and Fast-CDR resolved *inside* the prefix and stops if
they did not.

The plain archive links Flexiv's own Fast-RTPS and Fast-CDR, which only matters
for a binary that also links `rclcpp` — and `rt_hold_probe` does not; it is RDK plus
`iostream`. So the measurement this package exists to take can be made now,
while the ROS 2 integration waits on either the archive being fixed or the robot
software supporting v1.9.4+, where one self-contained `.so` makes the collision
impossible.

Check Flexiv's `basics1` against the standalone prefix first — if that also
crashes, the problem is not the Fast-DDS versions and the reproducer to send is
stronger still.

**`ModuleNotFoundError: No module named 'ament_package'`** while the dependency
script builds `foonathan_memory_vendor`.

A dependency tree first configured with ROS 2 sourced is being reused after the
environment was scrubbed. Flexiv's script clones into `thirdparty/cloned/` and
builds in place, so each dependency keeps a `CMakeCache.txt` that remembers the
paths it was configured with; `foonathan_memory_vendor` is an ament package, so
it still finds `/opt/ros/jazzy/share/ament_cmake_core` and then fails because
`PYTHONPATH` no longer reaches `ament_package`.

`install_rdk.sh` now records which variant built the tree and clears
`thirdparty/cloned/` when it changes, so this resolves itself on the next run.
To clear it by hand:

```bash
rm -rf ~/flexiv_rdk_standalone/thirdparty/cloned
```

If it recurs with a scrubbed environment and no stale clones, check the CMake
user package registry (`~/.cmake/packages/`), which `find_package` also
consults and which can point at ROS packages.

**`ignoring unknown package 'aico2_rt_control' in --packages-select`.**
`colcon` was run from somewhere other than the workspace root, so it saw no
`src/`. `cd ~/flexiv_ws` first.

**`ros2: command not found`.** ROS 2 is not sourced in that shell:
`source /opt/ros/jazzy/setup.bash && source install/setup.bash`.

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
| misses over 1% of cycles | not safe to drive the arm with as configured. Pin the loop to a core the stack does not use (`taskset`, or `isolcpus` at boot), move the perception stack off this host, or stay on NRT |
| fault during the loop | note what the robot reports; the probe only holds position, so a fault means the stream itself was rejected |

Run it twice: once with the rest of the stack stopped, once with Nav2 and the
cameras running. The difference between those two numbers is the thing that
decides whether RT is viable here, and no amount of reading the examples
answers it.
