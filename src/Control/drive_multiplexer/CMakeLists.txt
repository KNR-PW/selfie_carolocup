cmake_minimum_required(VERSION 3.0.2)
project(drive_multiplexer)

find_package(catkin REQUIRED COMPONENTS
  topic_tools
  roslint
)

catkin_package(
  CATKIN_DEPENDS topic_tools roslint
)

roslint_cpp()
roslint_python()
