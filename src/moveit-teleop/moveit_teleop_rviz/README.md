# moveit_teleop_rviz

One RViz panel, in C++: **ReplayPanel** — review a recorded teleoperation
episode without leaving the window the arm is being driven in.

| | |
|---|---|
| Plugin | `moveit_teleop_rviz/ReplayPanel` (`rviz_common::Panel`) |
| Subscribes | `/episode_player/status` (`moveit_teleop_msgs/ReplayStatus`) |
| Calls | `/episode_player/command` (`moveit_teleop_msgs/ReplayCommand`) |

It is stateless: the episode list, the playhead, the speed and the recorded mode
transitions all arrive on `ReplayStatus`, so the panel is correct the moment it
connects and needs no refresh button. Every interaction goes out as one
`ReplayCommand`, which keeps error handling in one place.

The panel is already in the stack's RViz config
(`moveit_teleop_bringup/config/teleop.rviz`) and in the replay package's own, so
there is normally nothing to add by hand. To add it to another config:
**Panels → Add New Panel → moveit_teleop_rviz → ReplayPanel**.

The ghost arm it scrubs is a `DisplayRobotState` display driven by
`moveit_teleop_replay`'s `episode_player`; see
[moveit_teleop_replay/README.md](../moveit_teleop_replay/README.md).
