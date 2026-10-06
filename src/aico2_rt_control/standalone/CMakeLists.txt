cmake_minimum_required(VERSION 3.16.3)
project(aico2_rt_control_standalone CXX)

# Builds rt_hold_probe with no ROS involvement at all.
#
# Why this exists: the RDK v1.9 "ros2-jazzy" prebuilt archive stack-smashes in
# the Robot constructor when linked against the Fast-DDS 2.14.6 / Fast-CDR 2.2.7
# that ROS 2 Jazzy currently ships. Flexiv's own basics1_display_robot_states
# fails the same way, so it is the archive-plus-dependency combination, not
# anything in this workspace.
#
# The plain archive with Flexiv's own vendored dependency versions avoids that.
# It statically links Fast-RTPS and Fast-CDR, which is only a problem for a
# binary that also links rclcpp -- and the probe does not: it is RDK plus
# iostream. So the measurement this package exists to take can be made now,
# while the ROS 2 integration waits on the archive being fixed or on the robot
# software supporting v1.9.4+ (one self-contained .so, no collision possible).
#
# Deliberately mirrors flexiv_rdk/example/CMakeLists.txt, so if this builds and
# the examples build, nothing here is the difference.
#
#   cmake -S . -B build -DCMAKE_PREFIX_PATH=$HOME/rdk_standalone
#   cmake --build build -j
#   ./build/rt_hold_probe Rizon4-063352
#
# Do NOT source ROS 2 before configuring this: the point is to keep ROS's
# Fast-DDS off the link line.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()
message(STATUS "build type ${CMAKE_BUILD_TYPE}")

find_package(flexiv_rdk REQUIRED)

add_executable(rt_hold_probe ../src/rt_hold_probe.cpp)
target_link_libraries(rt_hold_probe flexiv::flexiv_rdk)
set_target_properties(rt_hold_probe PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON)

# The static archive references spdlog and fmt without carrying them.
find_package(spdlog QUIET)
find_package(fmt QUIET)
if(spdlog_FOUND)
  target_link_libraries(rt_hold_probe spdlog::spdlog)
endif()
if(fmt_FOUND)
  target_link_libraries(rt_hold_probe fmt::fmt)
endif()
