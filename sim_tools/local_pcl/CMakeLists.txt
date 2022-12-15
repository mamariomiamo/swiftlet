cmake_minimum_required(VERSION 3.0.2)
project(local_pcl)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree filters)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  pcl_ros
)

catkin_package(
  CATKIN_DEPENDS roscpp pcl_ros
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(
  pcl_render_node
  src/local_pcl.cpp
)

target_link_libraries(pcl_render_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
