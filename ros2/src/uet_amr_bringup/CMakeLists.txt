cmake_minimum_required(VERSION 3.8)
project(uet_amr_bringup)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetMode.srv"
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(
  PROGRAMS scripts/amr_mode_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
