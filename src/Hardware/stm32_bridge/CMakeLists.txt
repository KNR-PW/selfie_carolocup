cmake_minimum_required(VERSION 3.0.2)
project(stm32_bridge)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  custom_msgs
  roslint
  tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS std_msgs custom_msgs
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
add_dependencies(stm32_bridge ${catkin_EXPORTED_TARGETS})

install(TARGETS stm32_bridge
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
