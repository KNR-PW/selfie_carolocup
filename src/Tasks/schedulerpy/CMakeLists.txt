cmake_minimum_required(VERSION 3.16)
project(schedulerpy)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   roslint
   std_msgs
   message_generation
   actionlib
   actionlib_msgs 
   custom_msgs
   common
)

catkin_package(
 CATKIN_DEPENDS actionlib custom_msgs message_runtime
   actionlib actionlib_msgs message_generation rospy std_msgs common
)

catkin_python_setup()
roslint_python()


catkin_install_python(PROGRAMS
  scripts/schedulerpy_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

