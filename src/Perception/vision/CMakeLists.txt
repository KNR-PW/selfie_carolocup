cmake_minimum_required(VERSION 3.0.2)
project(vision)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
  custom_msgs
  cv_bridge
  image_transport
  roslint
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  # TODO
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs visualization_msgs custom_msgs cv_bridge image_transport
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

roslint_cpp()

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
)

install(PROGRAMS
    scripts/make_hom_cut_file
    DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(vision  src/vision.cpp)
target_link_libraries(vision ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(vision ${catkin_EXPORTED_TARGETS})

install(TARGETS vision
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
