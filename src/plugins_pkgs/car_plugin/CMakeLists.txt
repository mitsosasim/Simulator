cmake_minimum_required(VERSION 3.5)
project(car_plugin)

find_package(gazebo REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(include ${Boost_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS} ${roscpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})

add_library(car_plugin SHARED src/carlikerobot_ros_plugin.cpp src/carlikerobot.cpp)

ament_target_dependencies(car_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} "std_msgs" "rclcpp")

ament_export_dependencies(rclcpp gazebo_ros std_msgs)
ament_package()

link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
