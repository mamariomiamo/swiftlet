cmake_minimum_required(VERSION 3.0.2)
project(px4ctrl)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  quadrotor_msgs
  roscpp
  std_msgs
  tf2
  tf2_ros
  px4_tf2
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs mavros_msgs quadrotor_msgs roscpp std_msgs tf2 tf2_ros px4_tf2
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(px4ctrl
  src/px4ctrl_main.cpp 
  src/px4fsm.cpp 
)

target_link_libraries(px4ctrl
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
