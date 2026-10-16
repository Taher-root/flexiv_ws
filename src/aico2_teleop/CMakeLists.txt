cmake_minimum_required(VERSION 3.8)
project(aico2_teleop)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/trajectory_adapter.py DESTINATION lib/${PROJECT_NAME})

ament_package()
