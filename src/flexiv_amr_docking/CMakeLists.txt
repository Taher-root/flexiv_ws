cmake_minimum_required(VERSION 3.8)
project(flexiv_amr_docking)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/detected_dock_pose_publisher.py
        DESTINATION lib/${PROJECT_NAME})
ament_package()
