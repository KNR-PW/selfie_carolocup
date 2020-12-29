cmake_minimum_required(VERSION 3.0.2)
project(free_drive)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
  std_msgs
  custom_msgs
  dynamic_reconfigure
  roslint
  common
)

find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(
   cfg/free_drive.cfg
)

roslint_cpp()

catkin_package(
  CATKIN_DEPENDS std_msgs actionlib_msgs custom_msgs actionlib roscpp dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

add_executable(free_drive src/free_drive_server.cpp src/free_drive.cpp)
target_link_libraries(free_drive ${catkin_LIBRARIES} commonlib)
add_dependencies(free_drive ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

install(TARGETS free_drive
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)