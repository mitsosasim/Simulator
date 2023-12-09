cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(gps_plugin)

find_package(gazebo REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(utils REQUIRED)

generate_messages(DEPENDENCIES )

include_directories(include ${GAZEBO_INCLUDE_DIRS})

add_library(gps_plugin SHARED src/gps_plugin.cpp include/gps_plugin.hpp)

ament_target_dependencies(gps_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} "utils")
ament_export_dependencies(gazebo_ros rclcpp utils)
ament_package()

link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
