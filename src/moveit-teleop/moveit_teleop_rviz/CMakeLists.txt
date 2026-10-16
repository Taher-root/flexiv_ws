cmake_minimum_required(VERSION 3.8)
project(moveit_teleop_rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(moveit_teleop_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# The panel derives from a QObject with signals and slots, so its header has to
# go through moc. Without this the class links but every connect() silently
# does nothing, which looks like an unresponsive UI rather than a build error.
set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES include/moveit_teleop_rviz/replay_panel.hpp)

add_library(${PROJECT_NAME} SHARED
  src/replay_panel.cpp
  ${MOC_FILES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rclcpp
  rviz_common
  moveit_teleop_msgs
)

# rviz_common looks the panel up through this, not through the library name.
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(pluginlib rclcpp rviz_common moveit_teleop_msgs)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
