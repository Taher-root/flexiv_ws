cmake_minimum_required(VERSION 3.16)
project(aico2_rt_control)

# Everything in this package that does NOT link the Flexiv RDK, which is
# everything except three programs. Built by colcon, so a plain `colcon build`
# produces the bridge, the shared-memory tools, the fake server and the tests
# with no separate cmake invocation.
#
# What is deliberately NOT here, and must not be added:
#
#   rt_server, rt_hold_probe, joint_map_probe   -- they link the RDK
#
# Those live in standalone/, built by their own plain CMake project against a
# prefix installed by scripts/install_rdk.sh --standalone. The separation is
# structural, not stylistic: a binary linking both rclcpp and the RDK pulls two
# incompatible eprosima versions into one address space, and the RDK archive
# built to share ROS 2's copies stack-smashes against the Fast-DDS that Jazzy
# ships. Worse, the RDK's find_package consults the *environment*
# CMAKE_PREFIX_PATH, so merely configuring it from a ROS-sourced shell -- which
# is what colcon is -- selects the wrong Fast-DDS silently. Two build systems
# makes that mistake impossible rather than merely discouraged. See README.md.
#
# The test and tool sources below are shared with two plain-CMake projects
# (test/ and standalone/) that build the same files without ROS present at all,
# for a machine with no ROS installation. Both paths are expected to work.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# rt_bridge -- the ROS half. Links rclcpp, nothing from the RDK.
# ---------------------------------------------------------------------------
add_executable(rt_bridge src/rt_bridge.cpp)
target_include_directories(rt_bridge PRIVATE include)
ament_target_dependencies(rt_bridge
  rclcpp rclcpp_action sensor_msgs std_msgs trajectory_msgs control_msgs std_srvs)
# shm_open/shm_unlink: in librt on glibc before 2.34, harmless after.
target_link_libraries(rt_bridge rt)
install(TARGETS rt_bridge DESTINATION lib/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Shared-memory tools and the fake server. No RDK, no ROS -- they speak only
# the shm protocol, so colcon can build them safely and `ros2 run` can find
# them. traj_publish and servo_publish drive a real arm through rt_server
# without ROS in the path; fake_server replaces the robot with a model, which
# is what makes the whole bridge path testable on a machine with no arm and no
# RDK installed.
# ---------------------------------------------------------------------------
foreach(tool traj_publish servo_publish)
  add_executable(${tool} src/${tool}.cpp)
  target_include_directories(${tool} PRIVATE include)
  target_link_libraries(${tool} Threads::Threads rt)
  install(TARGETS ${tool} DESTINATION lib/${PROJECT_NAME})
endforeach()

add_executable(fake_server test/fake_server.cpp)
target_include_directories(fake_server PRIVATE include)
target_link_libraries(fake_server Threads::Threads rt)
install(TARGETS fake_server DESTINATION lib/${PROJECT_NAME})

# The ROS-side twin of traj_publish: builds a goal rt_bridge will accept, which
# cannot reasonably be typed as YAML because it must cover every joint and start
# at the measured position.
install(PROGRAMS scripts/send_goal.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# ---------------------------------------------------------------------------
# Tests. These are where the correctness risk in this package actually lives --
# the sampler's maths, the servo tracker's limits, the shm protocol across a
# process boundary, the 1 kHz state machine -- and they need no robot, no RDK
# and no ROS. Registered with ament so `colcon test` runs them.
#
# No -Werror here, unlike test/CMakeLists.txt, which keeps it. A new compiler
# warning should fail a developer's test build, not block a robot from being
# rebuilt in the field.
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  foreach(t test_traj_sampler test_shm_protocol test_rt_executor test_traj_ingest
            test_servo_tracker)
    add_executable(${t} test/${t}.cpp)
    target_include_directories(${t} PRIVATE include)
    target_link_libraries(${t} Threads::Threads rt)
    ament_add_test(${t}
      COMMAND "$<TARGET_FILE:${t}>"
      TIMEOUT 120)
  endforeach()
endif()

ament_package()
