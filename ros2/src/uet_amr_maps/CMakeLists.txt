cmake_minimum_required(VERSION 3.8)
project(uet_amr_maps)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch maps DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS scripts/zones_manager.py scripts/clean_map.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_zones_manager test/test_zones_manager.py)
endif()

ament_package()
