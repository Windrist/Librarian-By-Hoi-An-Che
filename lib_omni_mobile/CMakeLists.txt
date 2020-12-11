cmake_minimum_required(VERSION 3.0.2)
project(lib_omni_mobile)

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

add_executable(omni_mobile src/OmniMobile.cpp)
target_link_libraries(omni_mobile ${catkin_LIBRARIES})

add_executable(convert src/convertTwist2Speed.cpp)
target_link_libraries(convert ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/controlKeyboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)