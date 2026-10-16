# moveit_teleop_msgs

The interface between the halves of the stack. No code — which is the point: a
device driver, a solver, a recorder or a UI can be written against these without
depending on MoveIt, a URDF or a controller.

Each `.msg` file carries the reasoning for its fields; this is the index.

## Messages

| Message | Direction | QoS it is published with | Purpose |
|---|---|---|---|
| `TeleopInput` | device → stack | BEST_EFFORT, depth 1 | pose, `pose_valid`, `clutch`, `gripper`. A continuous signal: the newest sample supersedes the last, so reliable delivery of stale poses would actively hurt |
| `TeleopEvent` | device → stack | RELIABLE, depth 10 | one discrete press — save, discard, home, estop, mark/return to a waypoint. Each matters exactly once; dropping "save this episode" loses a demonstration |
| `TeleopStatus` | mode manager → everyone | reliable | the single answer to "is it safe, is it moving, is it recording": `mode`, `recording`, `input_healthy`, `input_age`, `servo_status` |
| `TeleopFeedback` | stack → device | reliable | wall contact as a **level**: which walls, how deep, suggested haptic intensity. The only message that travels back down to the device |
| `IKStatus` | `ik_node` → diagnostics | reliable | `solve_time`/`solve_time_p99`, position and orientation error, `solver_status`, `limited_joints`, `rate_limited`, `collision_blocked`, `refused_jumps`, `retried_jumps` |
| `ReplayStatus` | `episode_player` → UI | reliable | the episode list, position, speed, and the recorded mode transitions, published continuously so a panel can be stateless |

Two design rules run through all of them:

- **Levels for continuous state, events for discrete actions.** `clutch` is a
  level: if a release were an event and the event were dropped, the arm would
  stay engaged forever. A level plus a timeout fails safe, because silence is
  indistinguishable from false.
- **`header.stamp` is when the device sampled, not when it published.**
  Consumers use it for staleness, and the dataset exporter aligns everything on
  it. A driver that stamps at publish time defeats both.

## Services

| Service | Served by | |
|---|---|---|
| `SetTeleopMode` | `mode_manager/set_mode` | request a transition. It may be refused — you cannot enter `TELEOP` with an unhealthy watchdog — so `success` is not a formality |
| `ReplayCommand` | `episode_player/command` | every player interaction in one service: `LOAD`, `PLAY`, `PAUSE`, `SEEK`, `STEP`, `SET_SPEED`, `SET_LOOP`, `REFRESH` |

Also used, from outside this package: `std_srvs/SetBool` for `ik_node/pause` and
for the hardware gate, and `moveit_msgs/{GetStateValidity, ApplyPlanningScene,
ServoCommandType}`.
