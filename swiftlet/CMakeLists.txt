cmake_minimum_required(VERSION 3.0.2)
project(swiftlet)


find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS
)

include_directories(
    ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

add_executable(mapping
  src/mapping.cpp
  )

target_link_libraries(mapping 
  ${catkin_LIBRARIES}
  )