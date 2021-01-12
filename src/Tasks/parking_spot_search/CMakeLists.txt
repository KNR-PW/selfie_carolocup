cmake_minimum_required(VERSION 3.0.2)
project(parking_spot_search)

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

add_executable(parking_spot_search
  src/parking_spot_search_server.cpp
  include/parking_spot_search/search_server.hpp
  src/parking_spot_search.cpp
)

add_dependencies(parking_spot_search ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(parking_spot_search
  ${catkin_LIBRARIES} commonlib
)

install(TARGETS parking_spot_search
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)