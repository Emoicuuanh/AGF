cmake_minimum_required(VERSION 3.0.2)
project(scan_safety)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  dynamic_reconfigure
  )

add_service_files(
  FILES
  SafetyControl.srv
)

generate_dynamic_reconfigure_options(
  cfg/SafetyScan.cfg
  cfg/Job0.cfg
  cfg/SafetyScanNg.cfg
  )

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  message_runtime
)
