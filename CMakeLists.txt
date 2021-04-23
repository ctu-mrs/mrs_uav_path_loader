cmake_minimum_required(VERSION 3.1.2)
project(path_loader)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  mrs_lib
  mrs_msgs
  )

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  #  INCLUDE_DIRS include
  LIBRARIES PathLoader
  CATKIN_DEPENDS roscpp mrs_lib mrs_msgs
  DEPENDS Eigen
  )

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

add_library(PathLoader
  src/path_loader.cpp)

add_dependencies(PathLoader
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(PathLoader
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )
