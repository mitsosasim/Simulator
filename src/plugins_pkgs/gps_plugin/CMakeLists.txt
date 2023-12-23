cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(gps_plugin)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(utils REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})
include_directories(include ${GAZEBO_INCLUDE_DIRS})

add_library(gps_plugin SHARED src/gps_plugin.cpp include/gps_plugin.hpp)
target_include_directories(gps_plugin PUBLIC include)

ament_target_dependencies(gps_plugin "gazebo_dev" "gazebo_ros" "rclcpp" "utils")
ament_export_dependencies(gazebo_ros gazebo_ros utils)
ament_package()
