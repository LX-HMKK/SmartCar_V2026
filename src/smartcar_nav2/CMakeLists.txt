cmake_minimum_required(VERSION 3.8)
project(smartcar_nav2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_smartcar_nav2_launch.py TIMEOUT 90)
endif()

ament_package()
