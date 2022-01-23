cmake_minimum_required(VERSION 2.8.3)
project(lidar_euclidean_cluster_detect)

# find_package(autoware_build_flags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  grid_map_cv
  grid_map_msgs
  grid_map_ros
  jsk_rviz_plugins
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
  autoware_msgs
  # vector_map_server
)
find_package(autoware_msgs REQUIRED)
find_package(OpenMP)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

generate_messages(
        DEPENDENCIES
        autoware_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    # vector_map_server
    grid_map_ros
    grid_map_cv
    grid_map_msgs
    jsk_rviz_plugins
    autoware_msgs
)


# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})

include_directories(
  include
  ${autoware_msgs_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
link_directories(${OpenCV_LIBRARY_DIRS})

#Euclidean Cluster
add_executable(lidar_euclidean_cluster_detect
  src/detect_obstacles.cpp
  src/merging_and_clustering.cpp
  src/prepare_data.cpp
  src/cluster.cpp
)

# #Euclidean Cluster
# add_executable(lidar_euclidean_cluster_detect
#   nodes/lidar_euclidean_cluster_detect/lidar_euclidean_cluster_detect.cpp
#   nodes/lidar_euclidean_cluster_detect/cluster.cpp
# )

find_package(CUDA)
find_package(Eigen3 QUIET)

target_link_libraries(lidar_euclidean_cluster_detect
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)


add_dependencies(lidar_euclidean_cluster_detect
  ${catkin_EXPORTED_TARGETS}
)

if(OPENMP_FOUND)
  set_target_properties(lidar_euclidean_cluster_detect PROPERTIES
    COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
    LINK_FLAGS ${OpenMP_CXX_FLAGS}
  )
endif()

# install(DIRECTORY include/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  lidar_euclidean_cluster_detect
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
