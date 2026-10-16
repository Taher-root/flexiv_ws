# Tuning guide

In order. Each step has a thing to **measure**, not a thing to feel — and every
number below came off this hardware rather than out of a paper.

---

## 1. Check the nominal posture before anything else

Launch and read one line from `ik_node`:

```
nominal posture from SRDF named state 'ready_pose', condition number 121.8,
joint-limit headroom 0.36 rad
```

Above 500 it warns, and you should believe it. Regularising toward an
ill-conditioned posture biases every solve toward a singularity.

| Nominal posture | Jacobian condition number |
|---|---|
| FR3 `ready_pose` | 8.2 |
| Piper `ready_pose` | 121.8 |
| Piper all-zeros ("home") | 29 644 |
| Piper joint mid-range | **34 111** |

Which is why "just use mid-range" is not a safe default: on the Piper, joint2
and joint3 nearly cancel into an almost fully extended elbow, with joint5 at
zero contributing a wrist singularity on top.

---

## 2. Set `weight_regularization` by DOF, then leave it

It is the **sole** source of steady-state tracking error. On a 6-DOF arm a pose
target fully determines the configuration up to discrete branches, so there is
no nullspace for a posture term to shape and it can only pull you off target:

| `w_reg` | max position error | max orientation error |
|---|---|---|
| 0.000 | 0.011 mm | 0.0008° |
| 0.005 | 0.584 mm | 0.048° |
| 0.020 | 2.272 mm | 0.187° |
| 0.100 | 10.09 mm | 0.840° |

**6-DOF: 0.005. 7-DOF: 0.02** — there a real nullspace exists and the term
shapes the elbow for free.

Do not reach for this to fix wrist behaviour; it cannot. At 0.6 the arm stops
following altogether (95 mm of tracking error).

---

## 3. Set the speed ceiling from the solver's own demand

`rate_limit_scale` is your main safety ceiling, so set it deliberately rather
than by feel. Start low (0.2 on an arm you have never driven), record an
episode, then replay it through the solver offline at several settings and look
at where clipping stops.

Measured on a 20 s Piper episode:

| `rate_limit_scale` | cap | frames clipped | tracking err p95 | peak demand |
|---|---|---|---|---|
| 0.35 | 1.75 rad/s | 5.5 % | 5.9 mm | 2.27 rad/s |
| **0.50** | 2.50 rad/s | 0.1 % | 5.4 mm | 2.97 rad/s |
| 1.00 | 5.00 rad/s | 0 % | 5.4 mm | 2.97 rad/s |

**Stop at the first value that stops clipping.** 0.5 and 1.0 produce
byte-identical output on this arm, because the solver never demanded more than
59 % of the model limit — so 1.0 buys nothing and only widens what a fault can
command. It also delivers an accepted 20° step in 67 ms instead of 200 ms.

Live check: `/ik_node/status` carries a `rate_limited` flag. Persistently true
during ordinary motion means you are below the solver's demand.

> **Note on acceleration.** Nothing enforces it. MoveIt declares
> `max_acceleration: 5.0 rad/s²`; the command stream reaches 81 rad/s² over
> 20 ms and 23 over 100 ms. A naive acceleration clamp was tried and removed —
> ramping velocity without braking in anticipation of arrival overshoots and
> rings (a step to 0.05 rad overshot by 44 mrad and crossed the target three
> times). Doing it properly means a jerk-limited online trajectory generator,
> i.e. Ruckig, which is installed but not wired in.

---

## 4. Fix the wrist with `weight_orientation`

If the arm moves joints 5 and 6 in odd circles during simple translations, that
is usually a **wrist singularity**, not redundancy: when joint5 crosses zero the
joint4 and joint6 axes align and counter-rotate with no tool effect. The
signature is `corr(dj4, dj6) ≈ −0.98`.

| `weight_orientation` | joint4 travel | joint6 travel | joint5 within 5° of zero |
|---|---|---|---|
| 10.0 | 0.70 deg/cm | 0.70 deg/cm | 4.4 % of episode |
| 0.5 | 0.42 deg/cm | 0.35 deg/cm | 10.7 % of episode |

Lowering it halves wrist churn but frees joint5 to wander *into* the singularity
rather than through it. **There is no setting that fixes both.** The real fix is
a per-joint regularisation weight pulling joint5 toward its nominal, which does
not exist yet — `IKWeights.regularization` would have to become a vector.

Regularisation is not the lever here: at the strength needed to hold joint5 away
from zero it destroys tracking (step 2).

---

## 5. Size `wall_clearance` against tracking error

**This is the one that bites.**

The wall is enforced on the **command**. The arm trails the command. On the
Piper the commanded tool never violated the wall once in 1695 samples — and the
real tool went **72 mm past it, eight times, and hit the table**, because the
clearance was 30 mm and the p95 tracking error is 115 mm.

Measured on that episode:

| | median | p95 | max | phase |
|---|---|---|---|---|
| `\|real − commanded tool\|` | 46 mm | 115 mm | 200 mm | ~140 ms |

The clearance had been sized from *static gripper geometry* — "tool z = +0.017
is clear, −0.015 is in contact via `gripper_link1/2`" — which is correct and
completely insufficient.

**Two ways out, and you want both eventually:**

1. Set `wall_clearance` above the p95. Costs usable height above the table.
2. Close the loop: run the robot bringup with `follow:=true` so TF, `move_group`
   and the retargeter's anchor track the real arm rather than the command echo.
   Then the clearance can come back down to the tool-geometry number.

To measure it for your arm: record an episode, compare your driver's measured
tool pose against FK of the command stream, take the p95.

---

## 6. Then, and only then, the feel knobs

| Symptom | Knob |
|---|---|
| Not enough reach per hand movement | Raise `translation_scale`. Safe above 1:1 on the IK path. |
| Rotation twitchy | Lower `rotation_scale`. At 1:1 a small unintended wrist roll is a large tool roll. |
| Hand tremor reaching the arm | Lower `smoothing_alpha` toward 0.3. Costs ~32 ms of phase; do not stack much more, the solver already runs a joint-space average. |
| Arm chatters between nearby solutions | Raise `weight_smoothness`. |
| Arm freezes near a surface | Check `jump_retry_smoothness` is not 0. The startup guard line reports `retry x<value>`. |

### Sluggish

Check *where* the lag is before touching anything. Measured on the Piper, stage
by stage:

```
retargeted target -> COMMAND  (solver + WMA + rate limiter)   lag  -36 ms   residual  3.2 mm
COMMAND           -> REAL arm (CAN driver + arm dynamics)     lag -140 ms   residual 29.0 mm
retargeted target -> REAL arm (end to end)                    lag -176 ms   residual 30.9 mm
```

Four fifths of the delay is the arm and the CAN path, and **no teleop parameter
reaches it**. `rate_limit_scale` can only affect the first row.
