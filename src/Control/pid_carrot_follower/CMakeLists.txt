cmake_minimum_required(VERSION 2.8.3)
project(pid_carrot_follower)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  ackermann_msgs
  rospy
  roscpp
  std_msgs
  nav_msgs
  roslint
  geometry_msgs
  pid
)

generate_dynamic_reconfigure_options(
  cfg/ControlCalculator.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure ackermann_msgs std_msgs nav_msgs geometry_msgs
)

roslint_cpp()
roslint_python()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
    scripts/const_float64.py
    scripts/control_calculator.py
    DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(cmd_creator src/cmd_creator.cpp)
target_link_libraries(cmd_creator ${catkin_LIBRARIES})
add_dependencies(cmd_creator ${pid_carrot_follower_EXPORTED_TARGETS})
