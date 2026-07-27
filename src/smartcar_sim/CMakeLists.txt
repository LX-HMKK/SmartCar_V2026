cmake_minimum_required(VERSION 3.8)
project(smartcar_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install directories
install(
  DIRECTORY
    launch
    config
    models
    worlds
    rviz
    scripts
    maps
  DESTINATION share/${PROJECT_NAME}
  PATTERN "geo_candidates*" EXCLUDE
  PATTERN "email_body.txt" EXCLUDE
  PATTERN "apply_candidate.py" EXCLUDE
  PATTERN "geometry_scan.py" EXCLUDE
  PATTERN "short_test.sh" EXCLUDE
)

# Install Python executables (ros2 run smartcar_sim odom_relay.py)
install(
  PROGRAMS
    scripts/odom_relay.py
    scripts/odom_combined_relay.py
    scripts/auto_train.py
    scripts/test_through_poses.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/generate_competition_field.py
    scripts/sim_cleanup.sh
    scripts/sim_start.sh
    scripts/wait_for_wslg.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
