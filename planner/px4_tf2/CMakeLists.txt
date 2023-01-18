cmake_minimum_required(VERSION 3.0.2)
project(px4_tf2)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  roscpp
  tf2
  tf2_ros
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs quadrotor_msgs roscpp tf2 tf2_ros
  DEPENDS Eigen3
  LIBRARIES px4_tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(px4_tf2
  src/px4_tf2.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Eigen3::Eigen 
)
