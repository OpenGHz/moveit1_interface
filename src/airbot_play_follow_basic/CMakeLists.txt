cmake_minimum_required(VERSION 3.1.3)
project(airbot_play_follow_basic)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/follow_joint_trajectory_server_basic.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)