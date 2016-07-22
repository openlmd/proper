cmake_minimum_required(VERSION 2.8.3)
project(proper_cloud)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  pcl_ros
  sensor_msgs
)

find_package(octomap REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES etna_cloud
#  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${OCTOMAP_INCLUDE_DIRS})

link_libraries(${OCTOMAP_LIBRARIES})

## Declare a cpp library
# add_library(etna_cloud
#   src/${PROJECT_NAME}/etna_cloud.cpp
# )

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

add_executable(cloud src/cloud.cpp)
target_link_libraries(cloud ${catkin_LIBRARIES})

add_executable(find_plane src/find_plane.cpp)
target_link_libraries(find_plane ${catkin_LIBRARIES})
