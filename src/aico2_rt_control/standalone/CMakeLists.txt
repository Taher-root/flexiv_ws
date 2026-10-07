cmake_minimum_required(VERSION 3.16.3)
project(aico2_rt_control_standalone CXX)

# Builds the RT server, probes and shm tools with no ROS involvement at all.
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
# Build:
#   cd standalone && mkdir -p build && cd build
#   cmake .. && make
#
# The RDK is found automatically. Search order:
#   1. CMAKE_PREFIX_PATH (if you installed the RDK to a prefix)
#   2. FLEXIV_RDK_DIR environment variable or cmake variable
#   3. Common sibling directories relative to this workspace
#
# Do NOT source ROS 2 before configuring this: the point is to keep ROS's
# Fast-DDS off the link line.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()
message(STATUS "build type ${CMAKE_BUILD_TYPE}")

# ---------------------------------------------------------------------------
# Find the Flexiv RDK: try installed package first, then source tree
# ---------------------------------------------------------------------------
find_package(flexiv_rdk QUIET)

if(NOT flexiv_rdk_FOUND)
  # Locate the RDK source tree. Check FLEXIV_RDK_DIR, then common locations.
  if(NOT FLEXIV_RDK_DIR)
    set(FLEXIV_RDK_DIR "$ENV{FLEXIV_RDK_DIR}")
  endif()

  if(NOT FLEXIV_RDK_DIR OR NOT EXISTS "${FLEXIV_RDK_DIR}/CMakeLists.txt")
    # Walk up from this CMakeLists.txt to the workspace root:
    #   standalone/ -> aico2_rt_control/ -> src/ -> flexiv_ws/
    get_filename_component(_WS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)

    set(_SEARCH_PATHS
      # Sibling to the workspace (e.g. ~/flexiv_rdk next to ~/flexiv_ws)
      "${_WS_ROOT}/../flexiv_rdk"
      # Under a flexivrobotics folder next to the workspace
      "${_WS_ROOT}/../flexivrobotics/flexiv_rdk"
      # Inside ~/Documents
      "$ENV{HOME}/Documents/flexiv_rdk"
      "$ENV{HOME}/Documents/flexivrobotics/flexiv_rdk"
      # Home directory
      "$ENV{HOME}/flexiv_rdk"
      "$ENV{HOME}/flexivrobotics/flexiv_rdk"
      # Installed prefix
      "$ENV{HOME}/rdk_standalone"
    )

    foreach(_path ${_SEARCH_PATHS})
      get_filename_component(_path "${_path}" ABSOLUTE)
      if(EXISTS "${_path}/CMakeLists.txt" AND EXISTS "${_path}/include/flexiv")
        set(FLEXIV_RDK_DIR "${_path}")
        break()
      endif()
    endforeach()
  endif()

  if(FLEXIV_RDK_DIR AND EXISTS "${FLEXIV_RDK_DIR}/CMakeLists.txt")
    message(STATUS "Using Flexiv RDK source tree: ${FLEXIV_RDK_DIR}")
    add_subdirectory("${FLEXIV_RDK_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/flexiv_rdk")
  else()
    message(FATAL_ERROR
      "Could not find flexiv_rdk. Searched:\n"
      "  - CMAKE_PREFIX_PATH (find_package)\n"
      "  - FLEXIV_RDK_DIR variable\n"
      "  - Common paths relative to workspace\n"
      "\n"
      "Fix: set -DFLEXIV_RDK_DIR=/path/to/flexiv_rdk or\n"
      "     set -DCMAKE_PREFIX_PATH=/path/to/rdk_install_prefix")
  endif()
else()
  message(STATUS "Found installed flexiv_rdk: ${flexiv_rdk_DIR}")
endif()

find_package(Threads REQUIRED)

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
target_link_libraries(rt_server Threads::Threads rt)

# Writes a trajectory into rt_server's shared memory. Needs no RDK at all --
# it only touches the mapping -- so it is built unconditionally.
add_executable(traj_publish ../src/traj_publish.cpp)
target_include_directories(traj_publish PRIVATE ../include)
target_compile_features(traj_publish PRIVATE cxx_std_17)
target_compile_options(traj_publish PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(traj_publish Threads::Threads rt)
install(TARGETS traj_publish DESTINATION bin)

# The servo twin of traj_publish: drives the latest-target stream instead of a
# trajectory, so the tracker can be measured on the arm with neither ROS nor
# MoveIt Servo running. Also needs no RDK.
add_executable(servo_publish ../src/servo_publish.cpp)
target_include_directories(servo_publish PRIVATE ../include)
target_compile_features(servo_publish PRIVATE cxx_std_17)
target_compile_options(servo_publish PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(servo_publish Threads::Threads rt)
install(TARGETS servo_publish DESTINATION bin)

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
