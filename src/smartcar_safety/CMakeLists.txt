cmake_minimum_required(VERSION 3.8)
project(smartcar_safety)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/smartcar_safety"
)

include_directories(include)

add_executable(safety_node_cpp src/safety_node.cpp src/guard.cpp)
target_include_directories(safety_node_cpp PRIVATE include)
ament_target_dependencies(safety_node_cpp
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs std_srvs)

install(TARGETS safety_node_cpp DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
