# Health of the full-IK solver. Published by moveit_teleop_ik's ik_node.
#
# Exists because "the arm feels wrong" is not diagnosable without numbers.
# These are the ones that distinguish a slow solve from a saturated joint from
# an operator simply outrunning the arm.

std_msgs/Header header

# True while the node is solving and commanding the controller.
bool active
bool paused

# Last solve, and a rolling high-water mark. If p99 approaches the frame
# period the solver is the bottleneck; if it does not, the problem is elsewhere.
float32 solve_time
float32 solve_time_p99

# How far the achieved pose is from the commanded one. A steady-state residual
# here is the cost weights, not a fault -- the posture term trades accuracy for
# a natural-looking configuration.
float32 position_error       # m
float32 orientation_error    # rad

# 'optimal', 'acceptable' (hit max_iter but usable), or 'failed: ...'
string solver_status

# Joints sitting on their bounds. With limits as CONSTRAINTS this should be
# rare and transient; persistently non-empty means the target is asking for
# something outside the workspace.
string[] limited_joints

# The candidate configuration was in collision, so the command is held at the
# last valid one (collision_policy 'stop').
bool collision_blocked

# Branch-flip solutions refused outright (the arm held position) and ones
# rescued by the retry pulled toward the last accepted solution. Counted since
# startup: a rising refused_jumps during ordinary motion means max_joint_jump
# is too tight or jump_retry_smoothness too weak.
uint32 refused_jumps
uint32 retried_jumps

# The commanded step was clipped to the joints' velocity limits, i.e. the
# operator is moving faster than the arm can follow.
bool rate_limited

string message
