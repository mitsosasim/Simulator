cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(traffic_light_plugin)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})
include_directories(include ${GAZEBO_INCLUDE_DIRS})

add_library(traffic_light_plugin SHARED src/traffic_light_plugin.cpp include/traffic_light_plugin.hpp)

ament_target_dependencies(traffic_light_plugin "gazebo_dev" "gazebo_ros" "rclcpp" "std_msgs")
ament_export_dependencies(rclcpp gazebo_ros)
ament_package()
