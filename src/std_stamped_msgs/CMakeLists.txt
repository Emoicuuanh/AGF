cmake_minimum_required(VERSION 3.0.2)
project(std_stamped_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib
  actionlib_msgs
)

add_service_files(
    FILES
    StringService.srv
)

add_message_files(
  DIRECTORY msg
  FILES
  BoolStamped.msg
  ByteStamped.msg
  ByteMultiArrayStamped.msg
  CharStamped.msg
  ColorRGBAStamped.msg
  DurationStamped.msg
  EmptyStamped.msg
  Float32Stamped.msg
  Float32MultiArrayStamped.msg
  Float64Stamped.msg
  Float64MultiArrayStamped.msg
  Int16Stamped.msg
  Int16MultiArrayStamped.msg
  Int32Stamped.msg
  Int32MultiArrayStamped.msg
  Int64Stamped.msg
  Int64MultiArrayStamped.msg
  Int8Stamped.msg
  Int8MultiArrayStamped.msg
  MultiArrayDimension.msg
  MultiArrayLayout.msg
  StringStamped.msg
  UInt16Stamped.msg
  UInt16MultiArrayStamped.msg
  UInt32Stamped.msg
  UInt32MultiArrayStamped.msg
  UInt64Stamped.msg
  UInt64MultiArrayStamped.msg
  UInt8Stamped.msg
  UInt8MultiArrayStamped.msg
)

add_action_files(
  FILES
  String.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  actionlib
  actionlib_msgs
)