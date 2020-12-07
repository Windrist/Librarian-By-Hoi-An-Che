cmake_minimum_required(VERSION 3.0.2)
project(lib_omni_mobile)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/controlOmniMobile.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
