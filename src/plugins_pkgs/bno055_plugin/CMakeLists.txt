cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(bno055_plugin)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(utils REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS} ${GAZEBO_LIBRARY_DIRS})
include_directories(include)
add_library(bno055_plugin SHARED src/bno055_plugin.cpp include/bno055_plugin.hpp)

ament_target_dependencies(bno055_plugin "gazebo_dev" "gazebo_ros" "utils")
ament_export_dependencies(gazebo_ros rclcpp utils)
ament_package()
