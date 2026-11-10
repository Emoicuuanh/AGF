cmake_minimum_required(VERSION 3.0.2)
project(mpc_agf_planner)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  nav_msgs
  base_local_planner
  cmake_modules
  costmap_2d
  nav_core
  nav_msgs
  pluginlib
  mbf_costmap_core
  mbf_msgs
  #safety_msgs
  #agv_common_library
  #agf_msgs
  agv_msgs
)
#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${CMAKE_MODULE_PATH}")
# add_message_files(
#   FILES
# )
generate_dynamic_reconfigure_options(
  cfg/MpcAgfPlannerReconfigure.cfg
  #...
)
find_package(Boost REQUIRED COMPONENTS system thread graph)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
# OSQP 0.6.x + osqp-eigen 0.8.x, installed system-wide (see the MPC setup guide)
find_package(OsqpEigen REQUIRED)
set(EXTERNAL_INCLUDE_DIRS  ${Threads_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
set(EXTERNAL_LIBS ${Boost_LIBRARIES} Threads::Threads)
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${EXTERNAL_INCLUDE_DIRS}
)

# add_service_files(
#   FILES
#
# )



catkin_package(
  INCLUDE_DIRS include ${EXTERNAL_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${EXTERNAL_LIBS}
  CATKIN_DEPENDS message_runtime geometry_msgs roscpp sensor_msgs std_msgs std_srvs tf2 tf2_ros pluginlib mbf_costmap_core
  DEPENDS
)

## Declare a C++ library


install(FILES lp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# ROS-free path tracking core (path reference, speed profile, MPC), unit-tested on its own
add_library(mpc_agf_core STATIC
  src/path_reference.cpp
  src/speed_profile.cpp
  src/mpc_controller.cpp
)
set_target_properties(mpc_agf_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(mpc_agf_core PUBLIC include ${EIGEN3_INCLUDE_DIR})
target_link_libraries(mpc_agf_core PUBLIC OsqpEigen::OsqpEigen)

add_library(mpc_agf_planner src/mpc_agf_planner.cpp)
add_dependencies(mpc_agf_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_agf_planner mpc_agf_core ${catkin_LIBRARIES} ${EXTERNAL_LIBS} Threads::Threads)
add_dependencies(mpc_agf_planner ${PROJECT_NAME}_gencfg)
install(TARGETS mpc_agf_planner
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

catkin_install_python(PROGRAMS script/identify_model.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_mpc test/test_mpc.cpp)
  if(TARGET test_mpc)
    target_link_libraries(test_mpc mpc_agf_core)
  endif()
endif()
