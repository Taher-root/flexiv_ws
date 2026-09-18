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

## 3. Order of operations

`src/aico2_left_arm_driver/scripts/rdk_primitive_pih.py` runs the stages.
Nothing that moves runs without `--yes-move`.

1. `check` — resting wrench and `info()`. No enable, no motion.
2. Declare the tool in Elements; re-run `check` until the resting wrench is
   small.
3. `zero` — `ZeroFTSensor`. The cheapest test that primitive execution works
   on this license at all. Documented against an F/T sensor we do not have,
   so a rejection is informative rather than a failure.
4. `search` — `SearchHole`, tool already in contact near the hole.
5. `checkpih` — `CheckPiH`.
6. `insert` — `InsertComp`.

Whether Adaptive Assembly is licensed here is unknown: the license string
reads `RDK-Professional`, and these primitives may be a separate package.
Step 3 or 4 will say — an unavailable primitive is rejected at
`ExecutePrimitive`, which is why the order above spends its first motion on
the cheapest primitive rather than on a spiral search.
