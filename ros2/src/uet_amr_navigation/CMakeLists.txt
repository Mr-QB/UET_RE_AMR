cmake_minimum_required(VERSION 3.8)
project(uet_amr_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(
  PROGRAMS scripts/cmd_vel_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
