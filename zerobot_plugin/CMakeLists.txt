cmake_minimum_required(VERSION 2.8.3)
project(zerobot_plugin)

set(CMAKE_CXX_STANDARD 11)

add_compile_options(-Wall -Wextra)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  nodelet
  roscpp
  roslint
  sensor_msgs
)

find_package(OpenCV REQUIRED)
message(STATUS OpenCV_VERSION)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES zerobot_plugin
  CATKIN_DEPENDS geometry_msgs nav_msgs nodelet roscpp roslint sensor_msgs
  DEPENDS Boost asio
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${asio_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  include/zerobot_plugin/bridge.h
  include/zerobot_plugin/teleop.h
  include/zerobot_plugin/wit_imu.h
  src/nodelet.cpp
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE ASIO_STANDALONE
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${asio_LIBRARIES}
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
