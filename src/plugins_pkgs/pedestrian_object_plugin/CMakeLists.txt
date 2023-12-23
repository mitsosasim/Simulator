cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(pedestrian_object_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS})

add_library(pedestrian_object_plugin SHARED
 src/geometric_point.cpp
 src/pedestrian_object_plugin.cpp 
 include/geometric_point.h
 include/pedestrian_object_plugin.h
)

target_include_directories(pedestrian_object_plugin PUBLIC include)
ament_target_dependencies(pedestrian_object_plugin gazebo_dev gazebo_ros rclcpp)
ament_package()
