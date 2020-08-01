cmake_minimum_required(VERSION 2.8.3)
project(selfie_basler_fisheye)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES selfie_camera
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

roslint_cpp()