cmake_minimum_required(VERSION 3.1.3)
project(airbot_play_launch)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY airbot_play_launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)