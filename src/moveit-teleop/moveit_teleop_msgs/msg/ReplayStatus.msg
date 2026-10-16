# State of the episode player, published continuously so a UI can be stateless.
#
# The available-episode list rides on this topic rather than sitting behind a
# service: a panel that subscribes is correct the moment it connects and stays
# correct when a new episode is recorded, with no polling and no refresh button
# to forget to press.

std_msgs/Header header

# Episode ids found in the episodes directory, newest first.
string[] episodes

# The loaded episode, or '' if none. Not necessarily episodes[0].
string episode
string task

# Position within the loaded episode.
int32 index
int32 count
float32 time                # seconds from episode start
float32 duration

bool playing
float32 speed               # 1.0 = recorded rate
bool loop

# Mode transitions, as recorded, so a UI can mark up the timeline: which parts
# of the episode were actually engaged rather than homing or halted.
float32[] mode_times        # seconds from episode start
uint8[] mode_values         # matches TeleopStatus.mode

string message              # last error, or '' when healthy
