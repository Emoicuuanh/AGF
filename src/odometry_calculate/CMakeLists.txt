cmake_minimum_required(VERSION 3.0.2)
project(odometry_calculate)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_filters
  nav_msgs
  rosconsole
  roscpp
  rospy
  tf2
  tf2_ros
  agv_common_library
)

find_package(Eigen3 REQUIRED)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
    CATKIN_DEPENDS
    dynamic_reconfigure # Must inside CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    std_msgs
    roscpp
    rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(odometry_forklift src/odometry_forklift.cpp)
target_link_libraries(odometry_forklift ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(odometry_forklift ${PROJECT_NAME}_generate_messages_cpp)

