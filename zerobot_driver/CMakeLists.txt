cmake_minimum_required(VERSION 2.8.3)
project(zerobot_driver)

set(CMAKE_CXX_STANDARD 11)

add_compile_options(-Wall -Wextra)

set(MESSAGE_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/msg.h)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  nav_msgs
  nodelet
  roscpp
  roslint
  sensor_msgs
  std_msgs
  tf2_ros
  zerobot_msgs
)

find_package(Boost REQUIRED COMPONENTS chrono system thread)

add_subdirectory(scripts)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES zerobot_driver
  CATKIN_DEPENDS diagnostic_msgs diagnostic_updater geometry_msgs nav_msgs nodelet roscpp roslint sensor_msgs std_msgs tf2_ros zerobot_msgs
  DEPENDS Boost
)

list(GET zerobot_msgs_MESSAGE_FILES 0 MESSAGE_FILE)
get_filename_component(MESSAGE_DIR ${MESSAGE_FILE} DIRECTORY)
file(GLOB MESSAGE_FILES ${MESSAGE_DIR}/*.msg)

genmsg(${MESSAGE_HEADER} ${MESSAGE_FILES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${MESSAGE_HEADER}
  include/zerobot_driver/diff_drive.h
  include/zerobot_driver/logger.h
  include/zerobot_driver/odometer.h
  include/zerobot_driver/params.h
  include/zerobot_driver/plugin.h
  include/zerobot_driver/ranger.h
  include/zerobot_driver/serial.h
  include/zerobot_driver/test.h
  include/zerobot_driver/twister.h
  src/nodelet.cpp
  src/plugin.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  zerobot_msgs_generate_messages
)

file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h)
file(GLOB SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

roslint_cpp(
  ${HEADERS}
  ${SRCS}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

catkin_add_gtest(${PROJECT_NAME}_test
  test/test.cpp
)

if(TARGET ${PROJECT_NAME}_test)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()
