cmake_minimum_required(VERSION 3.0.2)
project(common)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  geometry_msgs
  roscpp rospy
  roscpp_serialization
  custom_msgs
  std_msgs
  visualization_msgs
  roslint
  moveit_ros_planning_interface
  )

find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 3.8 REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS geometry_msgs std_msgs visualization_msgs custom_msgs roscpp moveit_ros_planning_interface
  INCLUDE_DIRS include
  LIBRARIES commonlib StatePublisherWrapper
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(commonlib
  src/${PROJECT_NAME}/marker_visualization.cpp
  src/${PROJECT_NAME}/state_publisher.cpp
)

## Declare a C++ -> Python3.8 wrappers library
add_library(StatePublisherWrapper 
  src/common/state_publisher_wrapper.cpp
)

## set C++ linker
set_target_properties(StatePublisherWrapper PROPERTIES LINKER_LANGUAGE CXX)

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

#link to wrappers library
target_link_libraries(StatePublisherWrapper 
  commonlib 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# Don't prepend wrapper library name with lib and add to Python libs.
set_target_properties(StatePublisherWrapper PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
        )