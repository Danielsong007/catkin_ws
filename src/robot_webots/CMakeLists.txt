cmake_minimum_required(VERSION 3.0.2)
project(robot_webots)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  message_generation
  geometry_msgs
)
## Generate services in the 'srv' folder
add_service_files(
  FILES
  NodeGetPosition.srv
  NodeGetPose.srv
  NodeGetOrientation.srv
  FieldSetVec3f.srv
  FieldSetRotation.srv
  SetSize.srv
  SetFloat.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# catkin_install_python(PROGRAMS scripts/joint_state_publisher.py
#         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# catkin_install_python(PROGRAMS scripts/trajectory_follower.py
#         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# catkin_install_python(PROGRAMS scripts/robots_ros.py
#         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# catkin_install_python(PROGRAMS scripts/webots_launcher.py
#         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
