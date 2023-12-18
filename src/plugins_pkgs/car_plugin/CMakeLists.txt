cmake_minimum_required(VERSION 3.5)
project(car_plugin)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces(
#   ${PROJECT_NAME}
#   DEPENDENCIES
#   std_msgs
# )

link_directories(${gazebo_dev_LIBRARY_DIRS} ${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS} ${roscpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})

add_library(car_plugin SHARED src/carlikerobot_ros_plugin.cpp src/carlikerobot.cpp)
target_include_directories(car_plugin PUBLIC include)
ament_target_dependencies(car_plugin "gazebo_dev" "gazebo_msgs" "gazebo_ros" "std_msgs" "rclcpp")

ament_export_dependencies(rclcpp gazebo_ros std_msgs)
ament_package()
