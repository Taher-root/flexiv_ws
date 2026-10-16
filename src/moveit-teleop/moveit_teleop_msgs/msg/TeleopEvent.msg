# A discrete, edge-triggered operator action.
#
# Published on ~/event with RELIABLE, depth 10 -- the opposite QoS from
# TeleopInput and for the opposite reason: every one of these matters exactly
# once. Dropping "save this episode" loses a demonstration; dropping one pose
# sample loses nothing.
#
# Kept in a separate message from TeleopInput so the two can have different
# QoS. Folding an event field into the high-rate input message would force a
# choice between dropping events and reliably delivering stale poses.

# --- Recording control (drives the episode recorder via the mode manager) ---
uint8 RECORD_START=0
uint8 RECORD_STOP=1
uint8 EPISODE_SAVE=2
uint8 EPISODE_DISCARD=3     # "delete the last take" -- the demonstrator's undo

# --- Mode and motion control ---
uint8 MODE_NEXT=4
uint8 HOME=5
uint8 ESTOP=6
# Remember the arm's current JOINT configuration, and plan back to it.
#
# A configuration rather than the tool pose it produces: asking a planner for a
# pose invites it to solve a different IK branch, so the arm would return to
# the right place with the elbow the other way round -- which is not the
# posture the operator marked.
uint8 MARK_WAYPOINT=7
uint8 PLAN_AND_GO=8         # hand off to move_group for a collision-free move

std_msgs/Header header

# One of the constants above.
uint8 type

# Optional free-form argument, e.g. a task label for RECORD_START.
# Deliberately a string rather than typed fields: it keeps this message stable
# as devices grow new buttons.
string payload
