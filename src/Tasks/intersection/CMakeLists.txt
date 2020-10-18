cmake_minimum_required(VERSION 3.0.2)
project(intersection)

 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp custom_msgs actionlib common dynamic_reconfigure roslint)

#roslint
set(ROSLINT_CPP_OPTS "--filter=-whitespace/ending_newline")
roslint_cpp()

find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(
  cfg/intersection_server.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES intersection
  CATKIN_DEPENDS custom_msgs
#  DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${intersection_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)

add_executable(intersection_server
  src/intersection_server.cpp
  include/intersection/intersection_server.hpp
  src/intersection_server_node.cpp
)
add_dependencies(intersection_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(intersection_server
  ${catkin_LIBRARIES} commonlib
)

install(TARGETS intersection_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)