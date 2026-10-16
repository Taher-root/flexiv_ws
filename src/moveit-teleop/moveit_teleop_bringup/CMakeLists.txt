cmake_minimum_required(VERSION 3.8)
project(moveit_teleop_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config doc
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

# Without this the tests in test/ are never run by `colcon test`: they pass
# when invoked by hand and report as zero tests in CI, which is how a launch
# argument referenced but never declared reached the device.
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_teleop_config test/test_teleop_config.py)
endif()

ament_package()
