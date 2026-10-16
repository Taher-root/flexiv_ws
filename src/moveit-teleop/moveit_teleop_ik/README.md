# moveit_teleop_ik

Full inverse kinematics, solved as a constrained optimisation **every frame**,
for teleoperation. Pose target in, joint trajectory out, 90 Hz.

Why not differential IK: a Jacobian step halts a joint at its limit and cannot
redistribute the motion, so the arm jams and the target runs away. Here the
joint limits and the virtual walls are *constraints* in the problem, so an
unreachable target yields the closest **feasible** pose and the arm reconfigures
instead of stopping.

```
PoseStamped target  ->  ik_node (90 Hz)  ->  JointTrajectory to the controller
                          NLP: limits + walls as constraints
```

The package is usable on its own: anything that publishes a `PoseStamped` and a
`move_group` to read the model from is enough. Signal flow and guard order:
[moveit_teleop_bringup/doc/architecture.md](../moveit_teleop_bringup/doc/architecture.md).

---

## Node: `ik_node`

| | |
|---|---|
| Subscribes | `target_topic` (`geometry_msgs/PoseStamped`), `joint_states_topic` (`sensor_msgs/JointState`) |
| Publishes | `<controller>/joint_trajectory` (`trajectory_msgs/JointTrajectory`), `~/status` (`moveit_teleop_msgs/IKStatus`) |
| Serves | `~/pause` (`std_srvs/SetBool`) — the mode manager's deadman |
| Calls | `collision_service` (`moveit_msgs/GetStateValidity`), asynchronously at `collision_check_rate` |
| Reads once | the URDF, SRDF and `joint_limits.yaml` from `description_source` |

### Parameters

| Parameter | Default | |
|---|---|---|
| `move_group` | `arm` | the SRDF group. Base link, tip link, joint order and the controller all follow from it |
| `description_source` | `/move_group` | whose parameters the robot model comes from |
| `nominal_state` | `ready_pose` | posture the solve is regularised toward (`midrange` is also accepted). Its Jacobian condition number is computed at startup and **WARNs above 500** |
| `controller` | `''` | `''` derives it from the controller map |
| `joint_states_topic` | `/control/joint_states` | on Piper hardware pass `/feedback/joint_states` — the default is a command bus there |
| `target_topic` | `~/pose_target_cmds` | |
| `solve_rate` | `90.0` | solves per second. Watch `solve_time_p99` against `1/solve_rate` |
| `command_time` | `0.0` | `time_from_start` on each point; `0` means one solve period |
| `weight_position` | `50.0` | position dominates the cost |
| `weight_orientation` | `10.0` | lower it to stop the wrist churning on simple translations |
| `weight_regularization` | `0.005` | the **sole** source of steady-state tracking error on a 6-DOF arm, where a pose fully determines the configuration. 0.02 on a 7-DOF arm is nearly free |
| `weight_smoothness` | `0.1` | penalises departure from the previous solution |
| `smoothing_weights` | `[0.4, 0.3, 0.2, 0.1]` | joint-space weighted moving average on the output |
| `rate_limit_scale` | `0.3` | fraction of each joint's MoveIt velocity limit teleop may use. **The main safety ceiling** |
| `use_moveit_limits` / `use_moveit_velocity_scaling` | `true` / `false` | where the limits come from; the second also applies MoveIt's own default scaling factor |
| `limit_margin` | `0.01` | rad kept away from each bound inside the solve |
| `max_iter` | `30` | IPOPT iterations; hitting it reports `acceptable`, not a failure |
| `max_joint_jump` | `0.35` | refuse a solution this far from the **previous solution** — an IK branch flip is valid kinematics and unacceptable motion |
| `jump_retry_smoothness` | `50.0` | on a refused jump, retry once pulled this many times harder toward the last accepted solution. `0` restores plain refusal |
| `target_timeout` | `0.5` | stop commanding when the target goes stale (the mode manager owns the actual halt) |
| `collision_policy` | `stop` | `stop` = full speed to contact, refuse to enter, hold the last valid command. `none` = no checking. Proximity *slowing* is deliberately not offered: working near a table means working deliberately close to geometry |
| `collision_check_rate` | `20.0` | Hz, asynchronous — the solve loop never blocks on the service |
| `collision_service` | `/check_state_validity` | |
| `scene_config` | `''` | the scene YAML; each box that declares a wall face becomes a constraint derived from that box's bounds, so it cannot drift from the collision object |
| `planes` | — | virtual walls given directly as a flat `[nx, ny, nz, d, …]`, for use without a scene file |

### Startup lines worth reading

```
group='arm' chain 'base_link' -> 'tcp_link' (6 joints) controller='arm_controller'
nominal posture from SRDF named state 'ready_pose', condition number 121.8
speed from MoveIt joint_limits.yaml x rate_limit_scale 0.5 -> [2.5 ...] rad/s
guards: joint limits from the model, jump limit 20 deg (retry x50), ...
```

The fourth line prints **every guard, with anything disabled at WARN**. Read it
before trusting any tuning measurement: a guard that reports off while it is in
fact firing invalidates the comparison.

---

## Library

Usable without ROS; the tests exercise all of it that way.

### `chain.SerialChain(urdf, base_link, tip_link)`

Analytic forward kinematics as a CasADi expression, so the Jacobian is exact
automatic differentiation rather than finite differences.

| Member | |
|---|---|
| `joint_names`, `nq`, `lower`, `upper`, `velocity` | the actuated chain, from the URDF |
| `forward(q)` | 4×4 pose |
| `jacobian_at(q)` | 6×nq, analytic |
| `condition_number(q)` | posture sanity check: FR3 `ready_pose` 8.2, Piper `ready_pose` 121.8, Piper mid-range 34111 |
| `midrange()`, `clamp(q)` | |
| `fk(q)` | the CasADi `Function` returning `(p, R)` — the symbolic FK the solver builds its NLP from |

### `ik_solver.IKSolver(chain, nominal, weights, …)`

| | |
|---|---|
| `solve(position, rotation, q_seed=None, active_planes=None, smoothness=None)` | → `IKResult(q, success, solve_time, position_error, orientation_error, status, limited_joints)` |
| `reset(q)` | reseed the warm start |
| `plane_violations(q)`, `_at_limits(q)` | diagnostics |
| `IKWeights(position, orientation, regularization, smoothness)` | the cost |

The NLP is built **once** with `opti.parameter()` for the target, so each frame
only changes numbers — there is no symbolic rebuild per solve. Orientation uses
a chordal (Frobenius) cost rather than `log3`, whose derivative is infinite at
zero error, which is exactly where a tracking controller lives.

### `smoothing`

`WeightedMovingAverage(weights)` and `JointRateLimiter(velocity, scale)`;
`would_limit(q, dt)` is what sets `rate_limited` on `IKStatus`.

### `collision.StateValidityMonitor`

Non-blocking `GetStateValidity` client: `offer(q)` fires a request, `valid`
holds the last verdict. Waiting on the service inside the solve loop would stall
the thing it protects.

## Tests

```bash
python3 -m pytest moveit_teleop_ik/test -q
```

FK against an independent oracle, the Jacobian against finite differences,
convergence, limit and wall constraints, branch-flip refusal and the retry,
and the rate limiter.
