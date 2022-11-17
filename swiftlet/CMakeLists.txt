cmake_minimum_required(VERSION 3.0.2)
project(swiftlet)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common octree filters)
find_package(octomap REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    octomap_msgs
    octomap_ros
    plan_env
)

catkin_package(
    CATKIN_DEPENDS plan_env
    DEPENDS Eigen3
)

include_directories(
    ${catkin_INCLUDE_DIRS} 
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(mapping
  src/mapping.cpp
  src/graph_search.cpp
  src/corridor_gen.cpp
  )

target_link_libraries(mapping 
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  )