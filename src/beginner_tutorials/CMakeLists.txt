cmake_minimum_required(VERSION 3.0.2)
project(beginner_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS scripts/talker.py scripts/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

