cmake_minimum_required(VERSION 3.0.2)
project(parking_spot_detector)

add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED COMPONENTS 
  geometry_msgs 
  roscpp rospy
  custom_msgs 
  std_msgs 
  visualization_msgs 
  actionlib 
  actionlib_msgs
  common 
  dynamic_reconfigure 
  roslint
  )

#roslint
set(ROSLINT_CPP_OPTS "--filter=-whitespace/ending_newline")
roslint_cpp( /src/ /include/)

generate_dynamic_reconfigure_options(
   cfg/detect_parking_spot.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES parking_spot_detector
  CATKIN_DEPENDS 
  actionlib_msgs 
  std_msgs 
  custom_msgs 
  visualization_msgs 
  geometry_msgs 
  roscpp
  actionlib
  dynamic_reconfigure
#  DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)

add_executable(detect_parking_spot
  python/src/search_parking_spot.py
  src/search_server.cpp
  include/parking_spot_detector/search_server.hpp
  src/detect_parking_spot.cpp
)

add_dependencies(detect_parking_spot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(detect_parking_spot
  ${catkin_LIBRARIES} commonlib
)

install(TARGETS detect_parking_spot
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)