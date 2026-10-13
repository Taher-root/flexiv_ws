# High-level supervisor mode (see docs/AICO2_ROS2_Plan.md §3.5).

uint8 IDLE=0
uint8 MANIPULATING=1
uint8 NAVIGATING=2
uint8 E_STOPPED=3
uint8 FAULT=4

uint8 state
string detail
