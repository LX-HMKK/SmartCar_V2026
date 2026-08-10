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

# The versioned field mask is rendered from the official geometry and shared
# by the editor and Gazebo.  Install a build-time copy with the production
# Nav2 package so the real stack can enforce field rules without treating it
# as a localization map or depending on a simulator installation.
set(SMARTCAR_KEEPOUT_MAP_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../smartcar_sim/maps"
)
set(SMARTCAR_KEEPOUT_MAP_FILES
  "${SMARTCAR_KEEPOUT_MAP_DIR}/field_map.pgm"
  "${SMARTCAR_KEEPOUT_MAP_DIR}/field_map.yaml"
)
foreach(SMARTCAR_KEEPOUT_MAP_FILE IN LISTS SMARTCAR_KEEPOUT_MAP_FILES)
  if(NOT EXISTS "${SMARTCAR_KEEPOUT_MAP_FILE}")
    message(FATAL_ERROR "Required field keepout artifact is missing: ${SMARTCAR_KEEPOUT_MAP_FILE}")
  endif()
endforeach()
install(FILES ${SMARTCAR_KEEPOUT_MAP_FILES}
  DESTINATION "share/${PROJECT_NAME}/maps"
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

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
  PATTERN "nav2_params_fixed.yaml" EXCLUDE
  PATTERN "behavior_trees" EXCLUDE
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_smartcar_nav2_launch.py TIMEOUT 90)
endif()

ament_package()
