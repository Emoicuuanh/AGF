cmake_minimum_required(VERSION 3.0.2)
project(agf_clearance_field_gazebo)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/scene_manager.py
  scripts/scenario.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
