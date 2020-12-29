cmake_minimum_required(VERSION 3.0.2)
project(scheduler)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roslint
  actionlib
  roscpp
  std_msgs
  custom_msgs
  topic_tools
  common
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS actionlib roscpp std_msgs custom_msgs topic_tools
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

roslint_cpp()

file(GLOB CLIENTS "src/clients/*.cpp")
add_executable(scheduler src/scheduler_node.cpp src/scheduler.cpp ${CLIENTS})
target_link_libraries(scheduler ${catkin_LIBRARIES} commonlib)
add_dependencies(scheduler ${catkin_EXPORTED_TARGETS})

add_executable(action_mock test/action_mock.cpp)
target_link_libraries(action_mock ${catkin_LIBRARIES} commonlib)
add_dependencies(action_mock ${catkin_EXPORTED_TARGETS})

add_executable(rc_mock test/rc_mock.cpp)
target_link_libraries(rc_mock ${catkin_LIBRARIES} commonlib)
add_dependencies(rc_mock ${catkin_EXPORTED_TARGETS})

install(TARGETS scheduler action_mock rc_mock
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
