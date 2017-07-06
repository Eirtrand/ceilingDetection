cmake_minimum_required(VERSION 2.8.3)
project(ceiling_detector)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pcl_tutorial
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)


## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



add_executable(ceiling_detector src/ceiling_detector.cpp)
target_link_libraries(ceiling_detector ${catkin_LIBRARIES})
