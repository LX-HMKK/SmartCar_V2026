cmake_minimum_required(VERSION 3.5)
project(rf2o_laser_odometry)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2  REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(Eigen3 REQUIRED)

set(EXECUTABLE_NAME "rf2o_laser_odometry_node")

add_executable(${EXECUTABLE_NAME}
    src/CLaserOdometry2DNode.cpp
    src/CLaserOdometry2D.cpp)

target_include_directories(${EXECUTABLE_NAME} PRIVATE
  include
)
target_link_libraries(${EXECUTABLE_NAME} Eigen3::Eigen)

set(DEPENDENCIES
  "rclcpp"
  "geometry_msgs"
  "tf2_geometry_msgs"
  "tf2"
  "tf2_ros"
  "sensor_msgs"
  "nav_msgs"
  "std_srvs"
)
ament_target_dependencies(${EXECUTABLE_NAME} ${DEPENDENCIES})

install(TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
