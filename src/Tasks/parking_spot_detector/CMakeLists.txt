cmake_minimum_required(VERSION 2.8.3)
project(parking_spot_detector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
  catkin REQUIRED COMPONENTS 
  geometry_msgs 
  roscpp rospy 
  custom_msgs 
  std_msgs 
  visualization_msgs 
  ackermann_msgs 
  actionlib 
  actionlib_msgs
  nav_msgs 
  ackermann_msgs 
  tf 
  common 
  dynamic_reconfigure 
  roslint
  )

#roslint
set(ROSLINT_CPP_OPTS "--filter=-whitespace/ending_newline")
roslint_cpp( /src/ /include/)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/detect_parking_spot.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES parking_spot_detector
  CATKIN_DEPENDS 
  nav_msgs 
  actionlib_msgs 
  std_msgs 
  custom_msgs 
  visualization_msgs 
  geometry_msgs 
  ackermann_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)

add_executable(detect_parking_spot
  src/search_server.cpp
  include/parking_spot_detector/search_server.hpp
  src/detect_parking_spot.cpp
)
add_dependencies(detect_parking_spot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(detect_parking_spot
  ${catkin_LIBRARIES} commonlib
)

add_executable(detect_parking_spot_mock_client
  test/Search_client_mock.cpp
  test/Search_client_mock.h
  test/detect_parking_spot_mock_client.cpp
)
add_dependencies(detect_parking_spot_mock_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(detect_parking_spot_mock_client
  ${catkin_LIBRARIES} commonlib
)
