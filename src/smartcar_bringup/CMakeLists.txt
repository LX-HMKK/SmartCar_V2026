cmake_minimum_required(VERSION 3.8)
project(smartcar_bringup)

# 纯 launch/config 包：无 C++/Python 库，仅安装 launch 与 config 到 share
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(system_contracts test/test_system_contracts.py)
  add_launch_test(test/test_system_smoke.py TIMEOUT 120)
endif()

ament_package()
