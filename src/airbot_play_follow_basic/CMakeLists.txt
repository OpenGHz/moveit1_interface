cmake_minimum_required(VERSION 3.1.3)
project(airbot_play_follow_basic)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/follow_joint_trajectory_server_basic.py
  scripts/gripper_control.py
  scripts/ik_control_server.py
  scripts/airbot_twins_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)