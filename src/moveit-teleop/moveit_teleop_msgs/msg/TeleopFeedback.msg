# Robot -> operator. The only message that travels back down to the device.
#
# Teleoperating against a surface you cannot feel is the hard part: the tool is
# sliding along a virtual wall and the operator's hand is somewhere below it,
# with nothing to say so. A buzz in the controller closes that loop.
#
# Sent as a LEVEL, not an event: `active` stays true for as long as contact
# lasts, so the device can hold a continuous rumble and stop when it clears.
# An edge-triggered pulse would need the device to guess how long to run, and
# a dropped edge would leave it buzzing forever.

std_msgs/Header header

bool active
# Which walls, by their name in the planning-scene config. Empty when inactive.
string[] sources
# How far behind the deepest wall the operator's hand has gone, in metres.
# A UI can scale intensity by this: light touch on approach, firm when
# pressing well past the surface.
float32 depth

# Suggested strength in [0, 1] and duration in seconds. Advisory: a device
# without haptics can ignore both and beep instead.
float32 intensity
float32 duration
