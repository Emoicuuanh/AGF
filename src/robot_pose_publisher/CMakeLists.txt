cmake_minimum_required(VERSION 3.0.2)
project(robot_pose_publisher)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  nav_msgs
  roscpp
  tf2
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    std_msgs
    nav_msgs
    roscpp
    tf2
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME} src/robot_pose_publisher.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(forklift_tf_publisher src/forklift_tf_publisher.cpp)
target_link_libraries(forklift_tf_publisher ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
