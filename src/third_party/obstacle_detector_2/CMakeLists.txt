cmake_minimum_required(VERSION 3.5)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
project(obstacle_detector)

# set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Armadillo REQUIRED)
find_package(Boost 1.54.0 REQUIRED system)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CircleObstacle.msg"
  "msg/Obstacles.msg"
  "msg/SegmentObstacle.msg"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces 
)

set(PROJECT_INCLUDE_DIRS include)
set(PROJECT_DEPENDS "rclcpp" "std_msgs" "nav_msgs" "std_srvs" "visualization_msgs" "geometry_msgs" "sensor_msgs" "tf2" "tf2_geometry_msgs" "tf2_ros" "laser_geometry" "rosidl_default_generators")

include_directories(include ${PROJECT_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

#
# Build libs
add_library(obstacle_extractor src/obstacle_extractor.cpp)
target_link_libraries(obstacle_extractor ${ARMADILLO_LIBRARIES})
ament_target_dependencies(obstacle_extractor ${PROJECT_DEPENDS})
# allow the project to link against its own messages
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(obstacle_extractor "${cpp_typesupport_target}")

add_library(obstacle_tracker src/obstacle_tracker.cpp)
target_link_libraries(obstacle_tracker ${ARMADILLO_LIBRARIES})
ament_target_dependencies(obstacle_tracker ${PROJECT_DEPENDS})
# allow the project to link against its own messages
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(obstacle_tracker "${cpp_typesupport_target}")

add_library(obstacle_publisher src/obstacle_publisher.cpp)
target_link_libraries(obstacle_publisher ${ARMADILLO_LIBRARIES})
ament_target_dependencies(obstacle_publisher ${PROJECT_DEPENDS})
# allow the project to link against its own messages
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(obstacle_publisher "${cpp_typesupport_target}")

#
# Build nodes
#
# add_executable(scans_merger_node src/nodes/scans_merger_node.cpp)
# target_link_libraries(scans_merger_node scans_merger)

add_executable(obstacle_extractor_node src/nodes/obstacle_extractor_node.cpp)
target_link_libraries(obstacle_extractor_node obstacle_extractor)

add_executable(obstacle_tracker_node src/nodes/obstacle_tracker_node.cpp)
target_link_libraries(obstacle_tracker_node obstacle_tracker)

add_executable(obstacle_publisher_node src/nodes/obstacle_publisher_node.cpp)
target_link_libraries(obstacle_publisher_node obstacle_publisher)

#
# Build nodelets
# 
# add_library(${PROJECT_NAME}_nodelets
#   src/nodelets/scans_merger_nodelet.cpp
#   src/nodelets/obstacle_extractor_nodelet.cpp
#   src/nodelets/obstacle_tracker_nodelet.cpp
#   src/nodelets/obstacle_publisher_nodelet.cpp)
# target_link_libraries(${PROJECT_NAME}_nodelets scans_merger obstacle_extractor obstacle_tracker obstacle_publisher)

# ament_export_targets(my_libraryTargets HAS_LIBRARY_TARGET)
# ament_export_dependencies(some_dependency)


#
# Install libraries
#
# install(TARGETS scans_merger obstacle_extractor obstacle_tracker obstacle_publisher ${PROJECT_NAME}_nodelets ${PROJECT_NAME}_gui
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS obstacle_extractor obstacle_tracker obstacle_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#
# Install nodes
#
# install(TARGETS scans_merger_node obstacle_extractor_node obstacle_tracker_node obstacle_publisher_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS obstacle_extractor_node obstacle_tracker_node obstacle_publisher_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

#
# Install header files
# #
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# #
# # Install nodelet and rviz plugins description
# #
# install(FILES nodelet_plugins.xml rviz_plugins.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#
# Create folders and copy resources
#
# file(MAKE_DIRECTORY $ENV{HOME}/.local/share/icons/robor)
# file(COPY resources/play.png resources/stop.png DESTINATION $ENV{HOME}/.local/share/icons/robor)

ament_package()