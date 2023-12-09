cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(pedestrian_object_plugin)

find_package(gazebo REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})

include_directories(include ${Boost_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS})

add_library(pedestrian_object_plugin SHARED
 src/geometric_point.cpp
 src/pedestrian_object_plugin.cpp 
 include/geometric_point.h
 include/pedestrian_object_plugin.h
)

ament_target_dependencies(pedestrian_object_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

