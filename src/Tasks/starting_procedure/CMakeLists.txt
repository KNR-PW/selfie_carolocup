cmake_minimum_required(VERSION 3.0.2)
project(starting_procedure)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  actionlib_msgs
  std_msgs
  dynamic_reconfigure
  roslint
  cv_bridge
  std_srvs
  nav_msgs
  custom_msgs
  tf
)

find_package(OpenCV 4 REQUIRED)
message(x ${OpenCV_VERSION})

generate_dynamic_reconfigure_options(
  cfg/starting_procedure.cfg
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  custom_msgs
  nav_msgs
  actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

roslint_cpp()
add_executable(starting_procedure src/starting_procedure_action.cpp src/starting_procedure.cpp)
target_link_libraries(starting_procedure ${catkin_LIBRARIES})
add_dependencies(starting_procedure ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(qr_decoder  src/qr_decoder.cpp src/decoder.cpp)
target_link_libraries(qr_decoder ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} zbar)
add_dependencies(qr_decoder ${catkin_EXPORTED_TARGETS})

add_executable(gate_scanner src/gate_scanner.cpp src/scanner.cpp)
target_link_libraries(gate_scanner ${catkin_LIBRARIES})
add_dependencies(gate_scanner ${catkin_EXPORTED_TARGETS})

install(TARGETS starting_procedure qr_decoder gate_scanner
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)