# RDK version options and the peg-in-hole path

Recorded because the answer to "can we upgrade RDK for RT control?" is not
what the RDK manual suggests, and the evidence is easy to lose.

Installed on the Jetson: `flexivrdk 1.9.0` (python3.12), robot software
`v3.11`, license `RDK-Professional`, arm `Rizon4`, `DoF=9` (7 arm + 2 waist),
`has_FT_sensor: False`.

## 1. Can we get RT control modes?

Not on this arm. Verified by importing each PyPI wheel off-robot and reading
what the bindings actually expose, rather than reading docs for a release we
do not run:

| flexivrdk | RT modes in `Mode` | `Stream*` methods |
| --------- | ------------------ | ----------------- |
| 1.7.0     | no                 | no                |
| 1.8.0     | no                 | no                |
| 1.9.0 (installed) | no         | no                |
| 1.9.3     | no                 | no                |
| 2.1.0     | `RT_JOINT_TORQUE`, `RT_JOINT_IMPEDANCE`, `RT_JOINT_POSITION`, `RT_CARTESIAN_MOTION_FORCE` | `StreamJointTorque`, `StreamJointPosition`, `StreamCartesianMotionForce` |

So RT lives only in 2.x — and 2.x is not for this robot. Its
`ProductModel` enum is `{Enlight-L, Enlight-LL, MICO-Core, MICO-Plus,
MICO-Ultra}`. There is no Rizon in it. RDK 2.x is the SDK for Flexiv's newer
product line, not a newer SDK for ours.

The RT modes do exist in the pre-2.x source (`mode.hpp` at tags v1.4-v1.5.1
declares all four), and the compiled 1.9.0 `.so` still contains the enum
name strings, but the Python binding does not register them. Whether Flexiv
can supply a Rizon build that exposes RT on robot software v3.11 is a
question for Flexiv support; nothing on PyPI does.

### 2.x is also a rewrite, not a bump

Every call is keyed by `JointGroup` in 2.x, so an upgrade would not be a
version pin change. Verified breaking differences against 1.9.0:

- `Enable()` → `ServoOn()`
- `LockExternalAxes()` **removed** — `arm_driver_node.py` depends on it to
  keep the waist still during teleop
- `ExecutePrimitive(name, dict)` → `ExecutePrimitive({JointGroup: PrimitiveArgs})`
- `SendJointPosition(...)`, `SendCartesianMotionForce(...)`,
  `SetCartesianImpedance(...)`, `SetForceControlAxis/Frame`,
  `SetMaxContactWrench`, `SetNullSpacePosture` all take a `JointGroup`
- `states().ext_wrench_in_tcp` → `tcp_wrench_local`,
  `ext_wrench_in_world` → `tcp_wrench`, `ft_sensor_raw` → `raw_ft_sensor`,
  `tcp_vel` → `tcp_twist`, `tau_des` gone
- `info().DoF_m` / `DoF_e` / `has_FT_sensor` / `model_name` gone, replaced by
  joint groups and `product_model` — the `q[0:2]` waist / `q[2:9]` arm split
  the waist driver relies on would have to be rederived

### RT would not buy us anything here anyway

The Cartesian impedance controller runs at 1 kHz *inside the robot* in NRT
mode too. RT exists so you can implement your own control law at 1 kHz on
the host. For peg-in-hole we want Flexiv's controller, not ours — and a
Jetson running ROS 2, nav2 and a three-peer DDS domain with no PREEMPT_RT
kernel is not a 1 kHz host.

## 2. The blocker for every force primitive: the ~24 N standing bias

`has_FT_sensor: False`, so every force number is estimated from the joint
torque sensors through the Jacobian. At rest, untouched, the arm reports
about `[-2.3, -23.9, 6.2] N` — roughly 24 N that is not there, almost
certainly the gripper's uncompensated mass.

The Adaptive Assembly primitives are all parameterised in newtons against
that same estimate (`SearchHole.contactForce` 5 ∈ [1..20],
`CheckPiH.searchForce` 3, `InsertComp.maxContactForce` 5). Ask SearchHole to
hold 5 N while the controller believes it is already pushing 24 N and it
backs off instead of pressing, then reports `lostContact`. No parameter
choice fixes this.

Fix: declare the tool (mass, centre of mass, inertia) in Flexiv Elements so
gravity compensation is correct, then confirm with
`rdk_primitive_pih.py <sn> check` that the resting wrench reads near zero.
Software biasing cannot substitute — it corrects what we print, never what
the controller believes.

## 3. Adaptive Assembly is not licensed on this robot

Confirmed on hardware, 2026-09-18. With the arm in Auto (Remote), enabled,
external axes locked and the mode switched to `NRT_PRIMITIVE_EXECUTION`,
`ExecutePrimitive("SearchHole", ...)` produced:

```
[error] [Event Log] [303009] Primitive [SearchHole] is unlicensed.
```

Everything up to the licence check worked: mode switch, `Enable()`, the waist
lock, the parameter encoding. Only the licence stops it. `info().license_type`
reads `RDK-Professional+TDK-Standard`, and Adaptive Assembly is evidently a
separate package.

Two things to know about how that failure presents:

- **`ExecutePrimitive` does not raise.** It returns normally and the refusal
  appears only in the controller event log. `rdk_primitive_pih.py` now proves
  the primitive actually started (`busy()` plus at least one of its own state
  keys) and reports `event_log()` when it did not, so an unlicensed primitive
  fails in two seconds naming the licence instead of polling to the watchdog.
- **The licence check happens at load, before motion.** That makes it safe to
  enumerate: `rdk_primitive_pih.py <sn> probe --yes-move` loads each primitive
  and `Stop()`s it immediately, and prints a licensed/unlicensed table.

## 4. What still works: the impedance API

The licence gated Flexiv's packaged search and insertion behaviour, not
compliance. `SetCartesianImpedance`, `SetForceControlAxis`,
`SetForceControlFrame`, `SetMaxContactWrench`, `SetNullSpacePosture`,
`SetPassiveForceControl` and `SendCartesianMotionForce` are RDK control API,
not primitives, and need no primitive licence.

`src/aico2_left_arm_driver/scripts/rdk_compliance_demo.py` uses them.
Configuration order matters, and comes from Flexiv's own
`intermediate4_non_realtime_cartesian_motion_force_control.py`:

1. `SwitchMode(NRT_CARTESIAN_MOTION_FORCE)`
2. read `states().tcp_pose` as the hold pose
3. `SetMaxContactWrench(<small>)` — soft-contact clamp while every axis is
   still motion-controlled
4. `SetForceControlFrame(...)`, `SetForceControlAxis([...])`
5. `SetMaxContactWrench([inf]*6)` **only after** force control is active on
   that axis, or the contact force spikes when the clamp lifts
6. `SendCartesianMotionForce(pose, wrench)` in a loop at 1-100 Hz

Motion control always references the world frame; force control can reference
world or TCP. Sign convention: in WORLD `+Fz` presses down, in TCP `-Fz`
presses along the tool. `SetCartesianImpedance` can be called inside the
streaming loop, so stiffness is ramped rather than stepped — stepping straight
to a low stiffness drops the arm.

Stages: `soft` (compliant hold — push it, it yields and springs back),
`press` (one axis force-controlled), `pih` (insertion axis force-controlled,
soft laterally and in rotation so the peg self-aligns).

### The bias sets a floor on how soft we can go

Impedance holds position against force, so a steady phantom force `F`
displaces the TCP by `F/K`. The ~24 N resting bias therefore sags the arm by
24/K metres as soon as the stiffness lands:

| stiffness | sag |
| --------- | --- |
| 10000 N/m (nominal) | 2.4 mm |
| 1000 N/m | 24 mm |
| 500 N/m | 50 mm |
| 200 N/m | 124 mm |

`rdk_compliance_demo.py` measures the resting wrench, computes that sag, and
refuses to ramp past `--max-sag` (default 50 mm). So until the tool is
declared in Elements, useful compliance bottoms out around 500 N/m — soft
enough to demonstrate (10 N gives 20 mm) but not soft enough for insertion.

`soft` works regardless: it commands no force, so the bias only sags it.
`press` and `pih` command a force the bias corrupts directly, and require
`--i-know-the-bias` to run at all.

### Joint impedance through the ROS driver, and why a blocked arm faults

Everything above is Cartesian. The `aico2_*_arm_driver` path is joint
impedance instead: `joint_control_mode:=impedance` puts the controller in
`NRT_JOINT_IMPEDANCE` and keeps streaming the same `SendJointPosition`
setpoints, so MoveIt trajectories and servo teleop are unchanged — the arm
simply yields to a push instead of fighting it. `joint_stiffness_ratio`
scales `K_q_nom` on the arm axes only (the waist entries are `+inf` and pass
through untouched).

Stiffness alone is not enough, and this is the trap. The impedance law
demands `K_q x deflection` with nothing bounding it, so an arm that is held
still keeps raising torque as the setpoint stream walks away from where the
arm actually is. Measured on this robot at `joint_stiffness_ratio:=0.15`
(K_q = 630 Nm/rad on arm joint 4), with the forearm blocked by hand:

| quantity | measured | torque demanded |
|---|---|---|
| RMS tracking lag | 3.500 deg | 38.5 Nm |
| peak tracking lag | 5.562 deg | 61.2 Nm |
| joint 4 limit | — | 64 Nm |

So it never saturates — it stops just short, and what actually trips is the
controller's collision detection, reported back as
`SendJointPosition failed: ... Minor fault occurred` and a `CONTROL_FAILED`
goal. Lowering the stiffness further does not fix it: it lowers the torque per
degree, but the setpoint stream keeps walking, so the deflection grows until
the same threshold is crossed.

`SetMaxContactTorque` is the knob that does fix it. RDK: *"the controller will
regulate its output to maintain contact torques with the environment under the
set values"* — it is a ceiling on applied torque, not an alarm level. It has
the same applicable modes as `SetJointImpedance`
(`RT_`/`NRT_JOINT_IMPEDANCE`), so the driver sets it immediately after the
mode switch, never before.

`max_contact_torque` (Nm, 0 = unset) exposes it:

```bash
ros2 launch flexiv_amr_bringup arms.launch.py \
    joint_control_mode:=impedance joint_stiffness_ratio:=0.15 \
    max_contact_torque:=10.0
```

10 Nm is roughly 25 N at the forearm — firm, but pushable. It is retunable
live (`ros2 param set /left_arm/left_arm_driver max_contact_torque 10.0`), and
the driver logs what actually landed:

```
joint stiffness set to 0.15 x K_q_nom on axes 2..8, max contact torque 10 Nm
```

A ceiling above an axis's own `tau_max` is clamped per axis rather than
rejected — the arm limits are 123/123/64/64/39/39/39 and
`SetMaxContactTorque` raises on anything above them, so a uniform request has
to be trimmed or nothing lands at all. The clamp is named in the same log
line. `torque UNBOUNDED` there means the parameter is still 0 and the fault
above is what to expect.

Two more things the same experiment showed:

- `joint_stiffness_ratio >= 0.8` is not compliance. At nominal `K_q` the arm
  is as stiff as position mode and reaches its torque limit in about a
  degree. The driver warns on configure if impedance mode is asked for at
  that ratio.
- Impedance tracking carries a steady-state error proportional to the
  unmodelled load — 1.7 deg of gravity sag was measured against a 1.15 deg
  position-mode goal tolerance, which makes convergence impossible by
  construction. Hence the separate `goal_joint_tolerance_impedance` (0.05 rad).

## 5. Order of operations

1. `rdk_primitive_pih.py <sn> check` — resting wrench and `info()`. No enable,
   no motion. Also the fastest check that the robot is in Auto (Remote):
   `IN_MANUAL_MODE` and even plain `IN_AUTO_MODE` block RDK control.
2. `rdk_compliance_demo.py <sn> soft --yes-move` — the arm becomes compliant
   and you can push it. Works today, licence and bias notwithstanding.
3. Declare the tool (mass, centre of mass, inertia) in Flexiv Elements;
   re-run step 1 until the resting wrench is near zero.
4. `rdk_compliance_demo.py <sn> pih --yes-move` at a low lateral stiffness —
   insertion without the primitives.
5. Only if Adaptive Assembly gets licensed: `rdk_primitive_pih.py <sn> probe`
   to confirm, then the `search` / `checkpih` / `insert` stages.
