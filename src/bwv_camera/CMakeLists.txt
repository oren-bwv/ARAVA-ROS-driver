cmake_minimum_required(VERSION 3.0.2)
project(bwv_camera)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-g)
add_compile_options(-O0)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  image_transport
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  bwv_msgs
  dynamic_reconfigure
)


generate_dynamic_reconfigure_options(
  cfg/BWVCamera.cfg
)


catkin_package(

)

include_directories(
  include
  include/bwv_camera
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  #/home/user/Documents/ControlCenter/sources/os

)

link_directories(
  ../../libs
)


add_executable(${PROJECT_NAME}_node 
src/bwv_camera_node.cpp
src/Main.cpp
src/VideoWrapper.cpp
src/GatingWrapper.cpp
src/utils.cpp

)

add_dependencies(${PROJECT_NAME}_node  ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  BWVAPI
  ${OpenCV_LIBS}
)
