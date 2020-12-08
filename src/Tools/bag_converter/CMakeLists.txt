cmake_minimum_required(VERSION 3.0.2)
project(bag_converter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  std_msgs
  custom_msgs
  topic_tools
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs custom_msgs roscpp topic_tools
)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)

add_executable(bag_converter src/bag_converter.cpp)

target_link_libraries(bag_converter
 ${catkin_LIBRARIES}
 )

install(TARGETS bag_converter
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS src/bag_converter.cpp
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/programs
)



