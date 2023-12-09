cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(traffic_light_plugin)

find_package(gazebo REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(traffic_light_pkg REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include ${GAZEBO_INCLUDE_DIRS})

add_library(traffic_light_plugin SHARED src/traffic_light_plugin.cpp include/traffic_light_plugin.hpp)

ament_target_dependencies(traffic_light_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} "std_msgs")

ament_export_dependencies(rclcpp gazebo_ros traffic_light_pkg)
ament_package()

link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
