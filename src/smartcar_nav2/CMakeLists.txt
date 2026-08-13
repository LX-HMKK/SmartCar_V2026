cmake_minimum_required(VERSION 3.8)
project(smartcar_nav2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
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
  config/behavior_trees/navigate_through_poses_return_w_replanning_and_recovery.xml
  config/behavior_trees/navigate_through_poses_transit_w_replanning_and_recovery.xml
  config/behavior_trees/navigate_through_poses_precise_w_replanning_and_recovery.xml
  config/behavior_trees/navigate_to_pose_precise_w_replanning_and_recovery.xml
  config/behavior_trees/navigate_to_pose_transit_w_replanning_and_recovery.xml
  config/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml
)
install(FILES ${SMARTCAR_NAV2_BEHAVIOR_TREE_FILES}
  DESTINATION "share/${PROJECT_NAME}/config/behavior_trees"
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "nav2_params_fixed.yaml" EXCLUDE
  PATTERN "behavior_trees" EXCLUDE
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_smartcar_nav2_launch.py TIMEOUT 90)
endif()

ament_package()
