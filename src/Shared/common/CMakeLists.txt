cmake_minimum_required(VERSION 3.0.2)
project(common)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  geometry_msgs
  roscpp rospy
  custom_msgs
  std_msgs
  visualization_msgs
  roslint
  )

catkin_package(
  CATKIN_DEPENDS geometry_msgs std_msgs visualization_msgs custom_msgs
  INCLUDE_DIRS include
  LIBRARIES commonlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(commonlib
  src/${PROJECT_NAME}/marker_visualization.cpp
  src/${PROJECT_NAME}/state_publisher.cpp
)

install(TARGETS commonlib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

roslint_cpp()

add_dependencies(commonlib
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(commonlib
  ${catkin_LIBRARIES}
)