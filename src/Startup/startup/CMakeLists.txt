cmake_minimum_required(VERSION 3.0.2)
project(startup)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  odometry
  intersection
  pid_carrot_follower
  starting_procedure
  obstacle_detector
  path_generator
  stm32_bridge
  park
  lane_detector
  free_drive
  scheduler
  drive_multiplexer
  lane_pilot
  diagnostics
  selfie_basler_fisheye
  tf
  parking_spot_detector
)

catkin_package()