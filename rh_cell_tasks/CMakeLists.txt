cmake_minimum_required(VERSION 3.16)
project(rh_cell_tasks)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(insertion_sequence src/insertion_sequence.cpp)
target_compile_features(insertion_sequence PUBLIC cxx_std_17)
ament_target_dependencies(insertion_sequence
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  shape_msgs
  std_msgs
)

install(TARGETS insertion_sequence DESTINATION lib/${PROJECT_NAME})

ament_package()
