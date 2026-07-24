cmake_minimum_required(VERSION 3.8)
project(smartcar_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ReadQr.srv"
  "srv/DescribeScene.srv"
  "srv/PrepareMotion.srv"
  "srv/ActivateMotion.srv"
  "srv/RenewMotion.srv"
  "srv/StopMotion.srv"
  DEPENDENCIES builtin_interfaces unique_identifier_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
