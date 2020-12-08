cmake_minimum_required(VERSION 3.0.2)
project(pid_carrot_follower)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  rospy
  roscpp
  std_msgs
  nav_msgs
  roslint
  geometry_msgs
  pid
  custom_msgs
)

generate_dynamic_reconfigure_options(
  cfg/ControlCalculator.cfg
  cfg/pidTuner.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure std_msgs nav_msgs geometry_msgs custom_msgs roscpp pid
)

roslint_cpp()
roslint_python()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
    scripts/const_float64.py
    scripts/control_calculator.py
    DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(cmd_creator src/cmd_creator.cpp)
target_link_libraries(cmd_creator ${catkin_LIBRARIES})
add_dependencies(cmd_creator ${pid_carrot_follower_EXPORTED_TARGETS})

add_executable(pid_tuner src/pid_tuner.cpp src/pid_tuner_node.cpp)
target_link_libraries(pid_tuner ${catkin_LIBRARIES})
add_dependencies(pid_tuner ${pid_carrot_follower_EXPORTED_TARGETS})

install(TARGETS cmd_creator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
