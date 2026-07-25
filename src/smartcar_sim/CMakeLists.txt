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
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
