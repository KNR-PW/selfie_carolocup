cmake_minimum_required(VERSION 3.0.2)
project(odometry)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  custom_msgs
  tf
  roslint
)


catkin_package(
  CATKIN_DEPENDS
  nav_msgs
  custom_msgs
  roscpp
  tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

roslint_cpp()

add_executable(odometry src/odometry.cpp src/odometry_node.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES})
add_dependencies(odometry ${catkin_EXPORTED_TARGETS} )

install(TARGETS odometry
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
