cmake_minimum_required(VERSION 3.0.2)
project(emotion_based_turtlebot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/autonomous_movement.py
  scripts/emotion_based_movement.py
  scripts/movement.py  # Add this line to install your script
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
