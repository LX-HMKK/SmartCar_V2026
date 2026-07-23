cmake_minimum_required(VERSION 3.8)
project(smartcar_nav2)

find_package(ament_cmake REQUIRED)

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

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_smartcar_nav2_launch.py TIMEOUT 90)
endif()

ament_package()
