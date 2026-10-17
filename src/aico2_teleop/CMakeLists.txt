cmake_minimum_required(VERSION 3.8)
project(aico2_teleop)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/trajectory_adapter.py
  scripts/vr_joystick_relay.py
  scripts/xr_bimanual_bridge.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
