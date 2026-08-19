cmake_minimum_required(VERSION 3.8)
project(uet_amr_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(scan_deskew_node src/scan_deskew_node.cpp)
ament_target_dependencies(scan_deskew_node
  rclcpp
  sensor_msgs
  laser_geometry
  tf2
  tf2_ros
)

install(
  TARGETS scan_deskew_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
