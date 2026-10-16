# What the teleop stack is currently doing. Published by the mode manager.
#
# This is the single place an operator, a recorder, or an RViz panel can look
# to answer "is it safe, is it moving, is it recording".

# --- Mode (see the mode manager state machine) ---
uint8 IDLE=0        # nothing engaged, servo paused
uint8 ALIGNED=1     # anchored to the device, ready to engage
uint8 TELEOP=2      # servo streaming from retargeted device input
uint8 PLANNING=3    # move_group executing; servo paused to avoid preempting it
uint8 HALTED=4      # fault: watchdog fired, estop, or servo halted

std_msgs/Header header

uint8 mode

# Recording is deliberately a flag rather than a mode: an episode can legitimately
# span both TELEOP and PLANNING, so it is orthogonal to the state machine.
bool recording
string episode_id

# Watchdog view of the device. input_age is seconds since the last input whose
# header.stamp was fresh; it is the number to plot when tuning the timeout.
bool input_healthy
float32 input_age

# Mirrors moveit_msgs/ServoStatus.code, or -1 if servo has not reported yet.
# Surfaced here so "the arm feels sticky" can be traced to collision or
# singularity deceleration without digging through another topic.
int8 servo_status

string message
