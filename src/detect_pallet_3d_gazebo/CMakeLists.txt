cmake_minimum_required(VERSION 3.0.2)
project(detect_pallet_3d_gazebo)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/block_laser_to_cloud2.py
  scripts/detection_eval.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/run_tilt_matrix.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch models rviz urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
