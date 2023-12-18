cmake_minimum_required(VERSION 3.5)
project(utils)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
    "msg/IMU.msg"
    "msg/Localisation.msg"
)
 
rosidl_generate_interfaces(
    ${PROJECT_NAME}
    ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
