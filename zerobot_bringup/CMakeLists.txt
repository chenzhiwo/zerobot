cmake_minimum_required(VERSION 2.8.3)
project(zerobot_bringup)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

catkin_package(
  CATKIN_DEPENDS roslaunch zerobot_driver zerobot_plugin
)

roslaunch_add_file_check(launch)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
