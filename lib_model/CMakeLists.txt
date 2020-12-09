cmake_minimum_required(VERSION 3.0.2)
project(lib_model)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})