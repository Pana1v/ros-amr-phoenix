cmake_minimum_required(VERSION 3.10)
project(joystick_to_cmdvel)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/omni.py
  scripts/diff.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
