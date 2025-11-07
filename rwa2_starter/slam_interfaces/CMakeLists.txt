cmake_minimum_required(VERSION 3.8)
project(slam_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Declare ROS 2 messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Keyframe.msg"
  DEPENDENCIES sensor_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
