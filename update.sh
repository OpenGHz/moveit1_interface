#!/bin/bash

# set the color variables of 'echo' cmd
DEF='\e[0m'
OK='\e[1;32m'
FAILED='\e[1;31m'
WARN='\e[1;33m'

# build the ros workspace
catkin clean -y 2> /dev/null
catkin build || { echo -e "${FAILED}ROS workspace build failed.${DEF}" && exit 0; }

echo -e "${OK}Update OK.${DEF}"