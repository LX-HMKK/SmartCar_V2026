cmake_minimum_required(VERSION 3.8)
project(smartcar_bringup)

# 纯 launch/config 包：无 C++/Python 库，仅安装 launch 与 config 到 share
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
