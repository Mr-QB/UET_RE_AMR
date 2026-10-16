cmake_minimum_required(VERSION 3.8)
project(uet_amr_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "msg/MapInfo.msg"
  "srv/SetMode.srv"
  "srv/SaveMap.srv"
  "srv/ListMaps.srv"
  "srv/GetMap.srv"
  "srv/SetTeleop.srv"
  "action/NavigateTo.action"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()