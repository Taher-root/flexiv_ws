cmake_minimum_required(VERSION 3.16)
project(aico2_rt_control)

# This is the ROS half and ONLY the ROS half: rt_bridge, which links rclcpp and
# nothing from the RDK.
#
# The RDK programs deliberately live in standalone/, built by their own plain
# CMake project. They are not reachable from colcon on purpose: a binary that
# links both rclcpp and the RDK pulls two incompatible eprosima versions into
# one address space, and the archive built to share ROS 2's copies stack-smashes
# against the Fast-DDS that Jazzy ships. Keeping them in separate build systems
# makes that mistake impossible rather than merely discouraged. See README.md.

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

add_executable(rt_bridge src/rt_bridge.cpp)
target_include_directories(rt_bridge PRIVATE include)
ament_target_dependencies(rt_bridge
  rclcpp rclcpp_action sensor_msgs trajectory_msgs control_msgs)
# shm_open/shm_unlink: in librt on glibc before 2.34, harmless after.
target_link_libraries(rt_bridge rt)

install(TARGETS rt_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
