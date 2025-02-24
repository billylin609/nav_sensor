cmake_minimum_required(VERSION 3.0.2)
project(vectornav_listener)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  vectornav
)

catkin_package()

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/ins_listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
