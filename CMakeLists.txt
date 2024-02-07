cmake_minimum_required(VERSION 2.8)
project(global_selfcalibration)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  uwb_localization_dwm # Should already built as a ROS package in the ROS PATH(before); to include UWB msg
)
# # Another way to directly add source in ROS package uwb_localization_dwm, and using its msg.
# include_directories(
#   # "/home/liu/workspace/ws/ros/xh_ws/devel/include/uwb_localization_dwm" # #include "UWBrange.h" // works
#   "/home/liu/workspace/ws/ros/xh_ws/devel/include" # #include "uwb_localization_dwm/UWBrange.h" // works
# )

find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package()

add_executable(global_selfcalibration_node
  src/globalOptNode.cpp
  src/globalOpt.cpp)

# Reqiured if non-official ROS packages(like uwb_localization_dwm) is include in building this source code;
# The ROS workspace of these packages should be included for building packsge in different workspaces.
# The add_dependencies() module should be after add_executable().
add_dependencies(global_selfcalibration_node ${catkin_EXPORTED_TARGETS} ${PCL_LIBRARIES})

target_link_libraries(global_selfcalibration_node ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${PCL_LIBRARIES})
