cmake_minimum_required(VERSION 2.8.3)
project(vision)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
  custom_msgs
  common
  roslint
)

generate_dynamic_reconfigure_options(
  cfg/vision.cfg
)

catkin_package(
  CATKIN_DEPENDS std_msgs custom_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

roslint_cpp()

add_executable(vision src/vision.cpp)
target_link_libraries(vision commonlib ${catkin_LIBRARIES})
add_dependencies(vision ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
