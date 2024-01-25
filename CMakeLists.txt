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
)

find_package(Ceres REQUIRED)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)


catkin_package()


add_executable(global_selfcalibration_node
	src/globalOptNode.cpp
	src/globalOpt.cpp)

target_link_libraries(global_selfcalibration_node ${catkin_LIBRARIES} ${CERES_LIBRARIES})