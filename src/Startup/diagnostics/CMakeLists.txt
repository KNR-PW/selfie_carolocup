cmake_minimum_required(VERSION 3.0.2)
project(diagnostics)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  roslint
)

catkin_package(
  CATKIN_DEPENDS
  sensor_msgs
  std_msgs
)

catkin_python_setup()

roslint_python()

install(PROGRAMS
  scripts/diagnose.py
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)