cmake_minimum_required(VERSION 3.0.2)
project(speaker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY sounds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/sounds
  USE_SOURCE_PERMISSIONS
)
