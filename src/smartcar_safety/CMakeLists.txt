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
find_package(smartcar_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/smartcar_safety"
)

include_directories(include)

add_executable(safety_node_cpp src/safety_node.cpp src/guard.cpp)
target_include_directories(safety_node_cpp PRIVATE include)
ament_target_dependencies(safety_node_cpp
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs std_srvs)

add_library(direction_guard_core src/direction_guard.cpp)
target_include_directories(direction_guard_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(direction_guard_node src/direction_guard_node.cpp)
target_include_directories(direction_guard_node PRIVATE include)
target_link_libraries(direction_guard_node direction_guard_core)
ament_target_dependencies(direction_guard_node
  rclcpp geometry_msgs nav_msgs smartcar_interfaces std_msgs
  unique_identifier_msgs)

install(TARGETS safety_node_cpp direction_guard_node direction_guard_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_direction_guard_cpp test/test_direction_guard.cpp)
  target_link_libraries(test_direction_guard_cpp direction_guard_core)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_direction_guard_contract_py
    test/test_direction_guard_contract.py)
endif()

ament_package()
