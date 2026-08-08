cmake_minimum_required(VERSION 3.8)
project(uet_amr_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
