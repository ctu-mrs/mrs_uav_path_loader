cmake_minimum_required(VERSION 3.8)
project(mrs_uav_path_loader)

############################################################
# зависимости пакета
############################################################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mrs_msgs REQUIRED)

############################################################
# include-директории
############################################################
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

############################################################
# исполняемые файлы
############################################################
add_executable(path_loader_node src/path_loader_node.cpp)
ament_target_dependencies(path_loader_node
  rclcpp
  Eigen3
  mrs_msgs
  )

add_executable(dummy_path_server src/dummy_path_server.cpp)
ament_target_dependencies(dummy_path_server
  rclcpp
  mrs_msgs)

############################################################
# установка
############################################################
install(TARGETS
  path_loader_node
  dummy_path_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params
DESTINATION share/${PROJECT_NAME})

ament_package()
