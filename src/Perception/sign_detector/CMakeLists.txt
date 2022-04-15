cmake_minimum_required(VERSION 3.0.2)
project(sign_detector)

find_package(catkin REQUIRED COMPONENTS
  custom_msgs
  cv_bridge
  image_transport
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS custom_msgs cv_bridge image_transport rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

 catkin_install_python(PROGRAMS
   scripts/sign_detector.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
