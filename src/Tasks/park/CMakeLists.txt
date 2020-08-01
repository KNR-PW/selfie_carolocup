cmake_minimum_required(VERSION 2.8.3)
project(park)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  actionlib_msgs
  geometry_msgs
  ackermann_msgs
  std_msgs
  dynamic_reconfigure
  custom_msgs
  common
  roslint
)

generate_dynamic_reconfigure_options(
  cfg/park.cfg
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  custom_msgs
  geometry_msgs
  actionlib_msgs
  ackermann_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

roslint_cpp()

add_executable(park src/park.cpp src/park_node.cpp)
target_link_libraries(park commonlib ${catkin_LIBRARIES})
add_dependencies(park ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
