cmake_minimum_required(VERSION 3.8)
project(smartcar_nav2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
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

add_library(smartcar_compute_reverse_path_through_poses_action_bt_node SHARED
  src/compute_reverse_path_through_poses_action.cpp
)
target_compile_definitions(smartcar_compute_reverse_path_through_poses_action_bt_node PRIVATE
  BT_PLUGIN_EXPORT
)
target_include_directories(smartcar_compute_reverse_path_through_poses_action_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(smartcar_compute_reverse_path_through_poses_action_bt_node
  reverse_path_utils
)
ament_target_dependencies(smartcar_compute_reverse_path_through_poses_action_bt_node
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

add_library(smartcar_compute_free_heading_path_action_bt_node SHARED
  src/compute_free_heading_path_action.cpp
)
target_compile_definitions(smartcar_compute_free_heading_path_action_bt_node PRIVATE
  BT_PLUGIN_EXPORT
)
target_include_directories(smartcar_compute_free_heading_path_action_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(smartcar_compute_free_heading_path_action_bt_node
  reverse_path_utils
)
ament_target_dependencies(smartcar_compute_free_heading_path_action_bt_node
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

add_library(smartcar_ackermann_reverse_retreat_action_bt_node SHARED
  src/ackermann_reverse_retreat_action.cpp
)
target_compile_definitions(smartcar_ackermann_reverse_retreat_action_bt_node PRIVATE
  BT_PLUGIN_EXPORT
)
target_include_directories(smartcar_ackermann_reverse_retreat_action_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(smartcar_ackermann_reverse_retreat_action_bt_node
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

add_library(smartcar_reverse_only_mppi_controller SHARED
  src/reverse_only_mppi_controller.cpp
)
target_include_directories(smartcar_reverse_only_mppi_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(smartcar_reverse_only_mppi_controller
  reverse_path_utils
)
ament_target_dependencies(smartcar_reverse_only_mppi_controller
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_mppi_controller
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros
)
pluginlib_export_plugin_description_file(
  nav2_core reverse_only_mppi_controller_plugin.xml
)

# Nav2 1.1.20 (TROS Humble): RewrittenYaml chain silently drops controller
# params in favour of YDLIDAR defaults, causing "No critics defined for
# FollowPath".  Work around by configuring a pre-resolved copy with the active
# install-prefix BT-XML paths that navigation_launch.py loads directly.
set(SMARTCAR_NAV2_SHARE_DIR
  "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}"
)
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/config")
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/config/nav2_params.yaml"
  "${CMAKE_CURRENT_BINARY_DIR}/config/nav2_params_fixed.yaml"
  @ONLY
)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config/nav2_params_fixed.yaml"
  DESTINATION "share/${PROJECT_NAME}/config"
)

set(SMARTCAR_NAV2_BEHAVIOR_TREE_FILES
  config/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml
  config/behavior_trees/navigate_through_poses_reverse_w_replanning_and_recovery.xml
  config/behavior_trees/navigate_through_poses_reverse_locked_w_replanning_and_recovery.xml
  config/behavior_trees/navigate_to_pose_precise_w_replanning_and_recovery.xml
  config/behavior_trees/navigate_to_pose_reverse_handoff_w_replanning_and_recovery.xml
  config/behavior_trees/navigate_to_pose_reverse_w_replanning_and_recovery.xml
  config/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml
)
install(FILES ${SMARTCAR_NAV2_BEHAVIOR_TREE_FILES}
  DESTINATION "share/${PROJECT_NAME}/config/behavior_trees"
)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
  PATTERN "nav2_params_fixed.yaml" EXCLUDE
  PATTERN "behavior_trees" EXCLUDE
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  reverse_path_utils
  smartcar_compute_reverse_path_to_pose_action_bt_node
  smartcar_compute_reverse_path_through_poses_action_bt_node
  smartcar_compute_free_heading_path_action_bt_node
  smartcar_ackermann_reverse_retreat_action_bt_node
  smartcar_reverse_only_mppi_controller
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

  ament_add_gtest(test_reverse_command_filter
    test/test_reverse_command_filter.cpp
  )
  ament_add_gtest(test_free_transit_goal_samples
    test/test_free_transit_goal_samples.cpp
  )
  ament_add_gtest(test_costmap_footprint_sweep
    test/test_costmap_footprint_sweep.cpp
  )
  ament_add_gtest(test_ackermann_reverse_retreat_path
    test/test_ackermann_reverse_retreat_path.cpp
  )
  target_include_directories(test_free_transit_goal_samples PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_include_directories(test_costmap_footprint_sweep PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_include_directories(test_ackermann_reverse_retreat_path PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_include_directories(test_reverse_command_filter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_reverse_path_utils reverse_path_utils)
  ament_target_dependencies(test_reverse_path_utils
    geometry_msgs
    nav_msgs
    tf2
    tf2_geometry_msgs
  )
  ament_target_dependencies(test_costmap_footprint_sweep
    geometry_msgs
    nav2_msgs
    nav_msgs
    tf2
    tf2_geometry_msgs
  )
  ament_target_dependencies(test_ackermann_reverse_retreat_path
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
ament_export_dependencies(
  nav2_core
  nav2_costmap_2d
  nav2_mppi_controller
  rclcpp_lifecycle
)
ament_package()
