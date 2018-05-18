cmake_minimum_required(VERSION 2.8.3)
project(zerobot_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  Encoder.msg
  Log.msg
  Odometry.msg
  Param.msg
  ParamInfo.msg
  Range.msg
  Topic.msg
  Twist.msg
  Wheel.msg
)

add_service_files(
  FILES
  ParamGet.srv
  ParamPull.srv
  ParamPush.srv
  ParamSet.srv
)

add_action_files(
  FILES
  ReturnHome.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime std_msgs
)
