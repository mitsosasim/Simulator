cmake_minimum_required(VERSION 3.0.2)
project(example)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  utils
  cv_bridge
  sensor_msgs
  tf
  geometry_msgs
  vision_msgs
)

catkin_package(
#  INCLUDE_DIRS include
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
  src/sign_detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

