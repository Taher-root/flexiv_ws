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

  # Which RDK version is installed changes what else has to be linked, so both
  # cases are handled rather than hard-coding one.
  #
  #   v1.9 .. v1.9.3  the library is a static archive that references spdlog and
  #                   fmt without carrying them, so a consumer must provide
  #                   both or the link fails with undefined spdlog::details::
  #                   log_msg::log_msg and fmt::v9::* symbols.
  #   v1.9.4 onwards  the library is a self-contained .so with its internal
  #                   symbols hidden; neither is needed.
  #
  # These arms run robot software v3.11, which rejects a v1.9.4.1 client
  # outright ("Version of this client is incompatible with robot"), so the
  # static-archive case is the live one. See README.md.
  find_package(spdlog QUIET)
  find_package(fmt QUIET)

  add_executable(rt_hold_probe src/rt_hold_probe.cpp)
  target_link_libraries(rt_hold_probe flexiv::flexiv_rdk)
  if(spdlog_FOUND)
    message(STATUS "aico2_rt_control: linking spdlog ${spdlog_VERSION}")
    target_link_libraries(rt_hold_probe spdlog::spdlog)
  endif()
  if(fmt_FOUND)
    message(STATUS "aico2_rt_control: linking fmt ${fmt_VERSION}")
    target_link_libraries(rt_hold_probe fmt::fmt)
  endif()

  install(TARGETS rt_hold_probe DESTINATION lib/${PROJECT_NAME})
else()
  # Print what was actually searched. The common cause is not a broken install
  # but the wrong machine: the RDK prefix is per host and per user, so a build
  # on a second machine, or as a different user, finds nothing even though the
  # first one works. Naming the host and the search path makes that obvious
  # instead of looking like a packaging fault.
  cmake_host_system_information(RESULT _rt_host QUERY HOSTNAME)
  message(WARNING
    "aico2_rt_control: flexiv_rdk (C++) not found -- skipping RT targets, so "
    "no rt_hold_probe will be installed and `ros2 run` will say \"No "
    "executable found\".\n"
    "  host:              ${_rt_host}\n"
    "  CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}\n"
    "  HOME:              $ENV{HOME}\n"
    "  Install the C++ RDK on THIS machine and pass its prefix, e.g.\n"
    "    colcon build --packages-select aico2_rt_control \\\n"
    "      --cmake-args -DCMAKE_PREFIX_PATH=$ENV{HOME}/rdk_install\n"
    "  The prefix is per host and per user -- installing it elsewhere does not "
    "count. See src/aico2_rt_control/README.md.\n"
    "  The Python flexivrdk wheel does NOT satisfy this: it has no RT modes.")
endif()

ament_package()
