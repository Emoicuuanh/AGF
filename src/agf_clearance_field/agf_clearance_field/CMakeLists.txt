cmake_minimum_required(VERSION 3.0.2)
project(agf_clearance_field)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/clearance_field_node.py
  scripts/fake_scan_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  if(COMMAND catkin_add_pytests)
    catkin_add_pytests(test/test_field_check.py)
  else()  # catkin < 0.8.13 (stock Noetic) has no pytest support
    catkin_add_nosetests(test/test_field_check.py)
  endif()
endif()
