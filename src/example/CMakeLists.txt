cmake_minimum_required(VERSION 3.0.2)
project(example)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  utils
  cv_bridge
  sensor_msgs
  tf2_ros
  geometry_msgs
  vision_msgs
  nav_msgs
  tf2_geometry_msgs
)

## (Optional) find Eigen for your EKF math
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs utils cv_bridge sensor_msgs tf2_ros geometry_msgs vision_msgs nav_msgs tf2_geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}      # for Eigen3
)

## Install your existing Python node
catkin_install_python(PROGRAMS
  src/sign_detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## -------------------------------------------------------------------
## Add your new C++ EKF fusion node
## -------------------------------------------------------------------

add_executable(ekf_fusion_node src/ekf_fusion_node.cpp)

target_link_libraries(ekf_fusion_node
  ${catkin_LIBRARIES}
)

## If you want to install the binary
install(TARGETS ekf_fusion_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
