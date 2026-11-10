cmake_minimum_required(VERSION 2.8.3)
project(agf_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation
)
add_service_files(
    FILES
    CheckPath.srv
    CheckJob.srv
    WaypointGlobal.srv
)
add_message_files(
    FILES
    Moving_Param.msg
    Safety_Control.msg
    SafetyStt.msg
    Waypoint.msg
    Waypoint_Array.msg
    Int8Array.msg
    Pallet_Mission.msg
    PLC_Input.msg
    StringParam.msg
    PoseParam.msg
)
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)
catkin_package(
    CATKIN_DEPENDS
    std_msgs
    message_runtime
    # geometry_msgs
)