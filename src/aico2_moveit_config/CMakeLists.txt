cmake_minimum_required(VERSION 3.8)
project(aico2_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# MoveItConfigsBuilder looks for this at the package share root, not in config/.
install(FILES
  .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/generate_srdf.py
  scripts/check_config.py
  scripts/moveit_goal.py
  scripts/check_collisions.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
