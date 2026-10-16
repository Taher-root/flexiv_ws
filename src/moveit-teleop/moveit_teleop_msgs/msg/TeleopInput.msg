# Normalized 6-DoF input from any teleoperation device.
#
# Published at device rate on ~/input with BEST_EFFORT, depth 1: this is a
# continuous signal where the newest sample always supersedes the last, so
# reliable delivery of stale poses would actively hurt.
#
# Everything here is device-side only. Nothing in this message knows or implies
# anything about a robot -- that is what lets a driver be written against it
# without depending on MoveIt, a URDF, or a controller.

# stamp   = when the device sampled this pose, NOT when it was published.
#           Consumers use it for staleness; a driver that stamps at publish
#           time defeats the watchdog.
# frame_id = the device's own reference frame (e.g. "webxr_local_floor").
#           Mapping this into a ROS frame is the driver's job, not core's.
std_msgs/Header header

# Raw 6-DoF device pose in the frame named by header.frame_id.
# Absolute, not a delta: clutching and relative accumulation are retargeting's
# concern, so devices stay stateless and replayable.
geometry_msgs/Pose pose

# False when tracking is lost or not yet acquired. Distinct from a stationary
# pose: "I don't know where the controller is" and "the controller is not
# moving" must never be conflated, or loss of tracking reads as hold-still.
bool pose_valid

# Deadman / engage. LEVEL-triggered, not edge-triggered: if this were an event
# and the release were dropped, the arm would stay engaged forever. A level
# plus a timeout fails safe, because silence is indistinguishable from false.
bool clutch

# Normalized gripper command, 0.0 = fully closed, 1.0 = fully open.
# Normalized so the same trigger drives a parallel jaw, a suction cup, or a
# hand; mapping to joint values comes from the SRDF on the robot side.
float32 gripper

# False if the device has no gripper axis, so consumers can hold last value
# rather than slamming the gripper to 0.0.
bool gripper_valid
