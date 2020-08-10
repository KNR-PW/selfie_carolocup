cmake_minimum_required(VERSION 3.0.2)
project(path_generator)

find_package(catkin REQUIRED COMPONENTS
  roslint
  rospy
  std_msgs
  nav_msgs
  custom_msgs
  geometry_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/PathGenerator.cfg
)

catkin_package(
  CATKIN_DEPENDS custom_msgs geometry_msgs roslint std_msgs nav_msgs dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

roslint_python()

catkin_install_python(PROGRAMS
    scripts/path_generator_node.py
    DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

