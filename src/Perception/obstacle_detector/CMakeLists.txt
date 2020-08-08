cmake_minimum_required(VERSION 3.0.2)
project(obstacle_detector)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamic_reconfigure
  custom_msgs
  common
  roslint
  tf
  geometry_msgs
  visualization_msgs
  sensor_msgs
)

generate_dynamic_reconfigure_options(
  cfg/obstacle_detector.cfg
)

catkin_package(
  CATKIN_DEPENDS 
  std_msgs 
  custom_msgs
  geometry_msgs
  visualization_msgs
  sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

roslint_cpp()

add_executable(obstacle_detector src/obstacle_detector.cpp src/obstacles_generator.cpp)
target_link_libraries(obstacle_detector commonlib ${catkin_LIBRARIES})
add_dependencies(obstacle_detector ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

install(TARGETS obstacle_detector
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
