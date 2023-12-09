cmake_minimum_required(VERSION 3.5)
project(utils)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 1 4)

 
rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/IMU.msg
        msg/localisation.msg
    DEPENDENCIES
        
)

 

include_directories(
# include
  
)
ament_package()
