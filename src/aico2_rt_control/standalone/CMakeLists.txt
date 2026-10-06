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
# It expects Flexiv's vendored Fast-DDS 2.6.10 / Fast-CDR 1.0.28, which it
# resolves dynamically. That is only a problem for a binary that also links
# rclcpp, since that would pull ROS 2's incompatible eprosima into the same
# address space -- and the probe does not link rclcpp: it is RDK plus iostream.
# So the measurement this package exists to take can be made now, while the ROS
# 2 integration waits on the ros2-jazzy archive being fixed or on the robot
# software supporting v1.9.4+ (one self-contained .so with hidden symbols, where
# the question cannot arise).
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

# Emit DT_RPATH rather than DT_RUNPATH, so transitive dependencies resolve.
#
# The RDK links Fast-RTPS, which in turn needs foonathan_memory; both live in
# the prefix. Modern linkers default to DT_RUNPATH, and ld.so applies an
# object's RUNPATH only to that object's OWN direct dependencies -- so the
# executable's RUNPATH finds libfastrtps.so but not the libfoonathan_memory.so
# that libfastrtps itself requires, which fails at startup with
# "cannot open shared object file". DT_RPATH is inherited down the chain and
# does resolve it. Verified both behaviours directly.
#
# Flexiv's example project does not do this, so their examples need
# LD_LIBRARY_PATH=<prefix>/lib at runtime instead; see README.md.
target_link_options(rt_hold_probe PRIVATE "-Wl,--disable-new-dtags")

# Read-only: reports the per-joint limits so the waist indices can be
# identified without commanding anything. See src/joint_map_probe.cpp.
add_executable(joint_map_probe ../src/joint_map_probe.cpp)
target_link_libraries(joint_map_probe flexiv::flexiv_rdk)
set_target_properties(joint_map_probe PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON)
target_link_options(joint_map_probe PRIVATE "-Wl,--disable-new-dtags")

# The RT server. Links the RDK and nothing from ROS, which is the whole point
# of the two-process split.
add_executable(rt_server ../src/rt_server.cpp)
target_include_directories(rt_server PRIVATE ../include)
target_link_libraries(rt_server flexiv::flexiv_rdk)
set_target_properties(rt_server PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON)
target_link_options(rt_server PRIVATE "-Wl,--disable-new-dtags")
find_package(Threads REQUIRED)
target_link_libraries(rt_server Threads::Threads rt)

# The static archive references spdlog and fmt without carrying them.
find_package(spdlog QUIET)
find_package(fmt QUIET)
foreach(tgt rt_hold_probe joint_map_probe rt_server)
  if(spdlog_FOUND)
    target_link_libraries(${tgt} spdlog::spdlog)
  endif()
  if(fmt_FOUND)
    target_link_libraries(${tgt} fmt::fmt)
  endif()
endforeach()
