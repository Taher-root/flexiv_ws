cmake_minimum_required(VERSION 3.8)
project(rizon10s_rt_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
