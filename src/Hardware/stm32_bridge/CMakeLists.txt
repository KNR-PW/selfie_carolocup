cmake_minimum_required(VERSION 3.0.2)
project(stm32_bridge)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  roscpp
  ackermann_msgs
  custom_msgs
  dynamic_reconfigure
  roslint
)

generate_dynamic_reconfigure_options(
  cfg/stm32_bridge.cfg
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs custom_msgs ackermann_msgs
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
)

roslint_cpp()

add_executable(stm32_bridge src/main.cpp src/usb.cpp src/bridge.cpp)
target_link_libraries(stm32_bridge ${catkin_LIBRARIES})
add_dependencies(stm32_bridge ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

install(TARGETS stm32_bridge
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
