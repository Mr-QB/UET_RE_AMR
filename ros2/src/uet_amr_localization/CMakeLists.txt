cmake_minimum_required(VERSION 3.8)
project(uet_amr_localization)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(imu_bias_corrector src/imu_bias_corrector.cpp)
ament_target_dependencies(imu_bias_corrector rclcpp sensor_msgs nav_msgs)
install(TARGETS imu_bias_corrector DESTINATION lib/${PROJECT_NAME})

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
