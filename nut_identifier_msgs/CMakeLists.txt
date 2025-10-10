cmake_minimum_required(VERSION 3.5)
project(nut_identifier_msgs)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NutPosition.msg"
  "msg/NutPositionArray.msg"
)

ament_package()
