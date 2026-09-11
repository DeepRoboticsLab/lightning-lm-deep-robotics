# Copyright (c) 2020 Livox. Message definitions only; sensor driver runs separately.
cmake_minimum_required(VERSION 3.16)
project(livox_ros_driver2)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
ament_index_register_resource("packages")
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
rosidl_generate_interfaces(livox_interfaces2
    "msg/CustomPoint.msg" "msg/CustomMsg.msg"
    DEPENDENCIES builtin_interfaces std_msgs
    LIBRARY_NAME ${PROJECT_NAME}
)
