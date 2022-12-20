cmake_minimum_required(VERSION 3.0.2)
project(localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  fast_gicp
  sensor_msgs
  nav_msgs
  tf
  laser_geometry
)

find_package(PCL 1.8 REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

###########
## Build ##
###########

add_executable(localization src/localization.cpp)
target_link_libraries(localization ${catkin_LIBRARIES})
target_link_libraries (localization ${PCL_LIBRARIES})


