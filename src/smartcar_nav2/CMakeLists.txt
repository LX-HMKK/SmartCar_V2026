cmake_minimum_required(VERSION 3.8)
project(smartcar_nav2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(reverse_path_utils STATIC
  src/reverse_path_utils.cpp
)
set_target_properties(reverse_path_utils PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
target_include_directories(reverse_path_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(reverse_path_utils
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

add_library(smartcar_compute_reverse_path_to_pose_action_bt_node SHARED
  src/compute_reverse_path_to_pose_action.cpp
)
target_compile_definitions(smartcar_compute_reverse_path_to_pose_action_bt_node PRIVATE
  BT_PLUGIN_EXPORT
)
target_include_directories(smartcar_compute_reverse_path_to_pose_action_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(smartcar_compute_reverse_path_to_pose_action_bt_node
  reverse_path_utils
)
ament_target_dependencies(smartcar_compute_reverse_path_to_pose_action_bt_node
  behaviortree_cpp_v3
  geometry_msgs
  nav2_behavior_tree
  nav2_msgs
  nav2_util
  nav_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# Nav2 1.1.20 (TROS Humble): RewrittenYaml chain silently drops controller
# params in favour of YDLIDAR defaults, causing "No critics defined for
# FollowPath".  Work around by providing a pre-resolved copy with hardcoded
# BT-XML paths that navigation_launch.py loads directly.
add_custom_target(generate_fixed_params ALL
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_CURRENT_SOURCE_DIR}/config/nav2_params.yaml"
    "${CMAKE_CURRENT_BINARY_DIR}/config/nav2_params_fixed.yaml"
  COMMENT "Generating nav2_params_fixed.yaml (RewrittenYaml bypass)"
)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config/nav2_params_fixed.yaml"
  DESTINATION "share/${PROJECT_NAME}/config"
)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
  PATTERN "nav2_params_fixed.yaml" EXCLUDE
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  reverse_path_utils
  smartcar_compute_reverse_path_to_pose_action_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_reverse_path_utils
    test/test_reverse_path_utils.cpp
  )
  target_link_libraries(test_reverse_path_utils reverse_path_utils)
  ament_target_dependencies(test_reverse_path_utils
    geometry_msgs
    nav_msgs
    tf2
    tf2_geometry_msgs
  )

  ament_add_pytest_test(reverse_navigation_contracts
    test/test_reverse_navigation_contracts.py
  )
  add_launch_test(test/test_smartcar_nav2_launch.py TIMEOUT 90)
endif()

ament_export_include_directories(include)
ament_package()
