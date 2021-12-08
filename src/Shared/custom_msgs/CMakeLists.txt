cmake_minimum_required(VERSION 3.0.2)
project(custom_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  actionlib_msgs
  std_msgs
  message_runtime
  roslint
  sensor_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Buttons.msg
  DriveCommand.msg
  Indicators.msg
  IntersectionStop.msg
  Motion.msg
  Box2D.msg
  Box2DArray.msg
  Box3D.msg
  Box3DArray.msg
  RoadLines.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
 add_action_files(
   DIRECTORY
   action
   
   FILES
   search.action
   driving.action
   starting.action
   intersection.action
   park.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs

  actionlib_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include 
  CATKIN_DEPENDS geometry_msgs message_runtime actionlib_msgs std_msgs sensor_msgs
)
set(ROSLINT_CPP_OPTS "--filter=-whitespace/braces")
roslint_cpp()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)