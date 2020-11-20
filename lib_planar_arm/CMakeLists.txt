cmake_minimum_required(VERSION 3.0.2)
project(lib_planar_arm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/controlPlanarArm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
