cmake_minimum_required(VERSION 3.0.2)
project(example)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  utils_ros
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
  CATKIN_DEPENDS roscpp rospy std_msgs utils_ros cv_bridge sensor_msgs tf2_ros geometry_msgs vision_msgs nav_msgs tf2_geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}      # for Eigen3
  ${utils_ros_INCLUDE_DIRS}   # ← usually same as catkin_INCLUDE_DIRS
)

## Install your existing Python node
catkin_install_python(PROGRAMS
  src/sign_detector_node.py
  src/robot_localization.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## -------------------------------------------------------------------
## Add your new C++ EKF fusion node
## -------------------------------------------------------------------

add_executable(ekf_fusion_node src/ekf_fusion_node.cpp)
add_executable(robot_localization src/robot_localization.cpp)

add_dependencies(ekf_fusion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(robot_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ekf_fusion_node
  ${catkin_LIBRARIES}
  ${utils_ros_LIBRARIES}   
)

target_link_libraries(robot_localization
  ${catkin_LIBRARIES}
  ${utils_ros_LIBRARIES}   
)

## If you want to install the binary
install(TARGETS ekf_fusion_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS robot_localization
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
