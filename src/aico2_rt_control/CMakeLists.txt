cmake_minimum_required(VERSION 3.16)
project(aico2_rt_control)

# C++17 to match the RDK headers and ROS 2 Jazzy.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------------------
# The RDK C++ library is NOT the flexivrdk Python wheel.
#
# The wheel installed on the Jetson gives Python the NRT modes only. RT lives
# in the C++ library, which is a prebuilt static archive Flexiv publishes per
# platform -- libflexiv_rdk.aarch64-linux-gnu.a for this machine -- and which
# the RDK's own CMake downloads from its GitHub release at configure time.
#
# Build it with RDK_SUPPORT_ROS2_JAZZY=ON. That selects
# libflexiv_rdk.aarch64-linux-gnu.ros2-jazzy.a instead, which exists because
# the RDK statically links Fast-RTPS and Fast-CDR and so does ROS 2: without
# the ROS 2 build you get duplicate-symbol or ODR problems the moment this
# links against rclcpp. Flexiv ships that variant precisely so the two can
# coexist, which is good evidence the ROS 2 case is anticipated.
#
# See README.md for the install steps. Until that is done this package
# configures to nothing and the build is a no-op rather than an error, so it
# cannot break a colcon build of the rest of the workspace.
# ---------------------------------------------------------------------------
find_package(flexiv_rdk QUIET)

if(flexiv_rdk_FOUND)
  message(STATUS "aico2_rt_control: flexiv_rdk found, building RT targets")

  # From v1.9.4.1 the RDK is a shared library living in its own install prefix
  # (e.g. ~/rdk_install/lib), which is not on the default loader path. Baking
  # that directory into the binary's RPATH is what makes `ros2 run` work without
  # LD_LIBRARY_PATH -- and setting LD_LIBRARY_PATH instead is actively harmful
  # here, because assigning it rather than appending drops ROS 2's own lib
  # directory and breaks rclpy with "librcl_action.so: cannot open shared object
  # file".
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

  add_executable(rt_hold_probe src/rt_hold_probe.cpp)
  target_link_libraries(rt_hold_probe flexiv::flexiv_rdk)

  install(TARGETS rt_hold_probe DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING
    "aico2_rt_control: flexiv_rdk (C++) not found -- skipping RT targets.\n"
    "  This is expected until the C++ RDK is installed; see "
    "src/aico2_rt_control/README.md.\n"
    "  The Python flexivrdk wheel does NOT satisfy this: it has no RT modes.")
endif()

ament_package()
