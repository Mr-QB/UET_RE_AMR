cmake_minimum_required(VERSION 3.8)
project(uet_amr_perception)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS scripts/mask_camera_info.py scripts/nvblox_footprint_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
