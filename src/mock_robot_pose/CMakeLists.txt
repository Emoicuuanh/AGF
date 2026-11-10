cmake_minimum_required(VERSION 3.0.2)
project(mock_robot_pose)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/mock_pose_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
