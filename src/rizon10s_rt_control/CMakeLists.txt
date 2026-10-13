cmake_minimum_required(VERSION 3.16)
project(rizon10s_rt_control)

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
target_link_libraries(rt_bridge rt)
install(TARGETS rt_bridge DESTINATION lib/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Shared-memory tools and the fake server.
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

install(PROGRAMS scripts/send_goal.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Tests
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
