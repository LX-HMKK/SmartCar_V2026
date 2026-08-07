cmake_minimum_required(VERSION 3.8)
project(smartcar_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install only assets consumed by the runtime launch. Development and
# verification tools remain source-tree utilities so an installed simulator
# does not expose one-off runners as supported entry points.
install(
  DIRECTORY
    launch
    config
    models
    worlds
    rviz
    maps
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# Runtime nodes launched by sim.launch.py. auto_train is intentionally kept
# because run_route:=true is an explicit supported verification mode.
install(
  PROGRAMS
    scripts/gazebo_ground_truth_odom_relay.py
    scripts/ackermann_arc_probe.py
    scripts/sim_sensor_preflight.py
    scripts/sim_perception_monitor.py
    scripts/sim_depth_pointcloud_adapter.py
    scripts/nav2_lifecycle_startup.py
    scripts/auto_train.py
  DESTINATION lib/${PROJECT_NAME}
)

# The local entry point loads these from the installed package. All other
# scripts are source-tree development tools and are intentionally not installed.
install(
  PROGRAMS
    scripts/sim_cleanup.sh
    scripts/sim_env.sh
    scripts/sim_start.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
