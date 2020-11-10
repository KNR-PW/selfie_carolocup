cmake_minimum_required(VERSION 3.0.2)
project(realsense_obstacle_detector)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  pcl_conversions
  pcl_ros
  roscpp
  roslint
  rospy
  sensor_msgs
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/realsense_obstacle_detector.cfg
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS pcl_ros pcl_conversions roscpp rospy sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

roslint_cpp()

add_executable(${PROJECT_NAME} src/main.cpp src/realsense_obstacle_detector/RealSensePcl.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(realsense_obstacle_detector ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
