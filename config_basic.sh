#!/bin/bash

# set the color variables of 'echo' cmd
DEF='\e[0m'
OK='\e[1;32m'
WARN='\e[1;33m'

# install useful robot tools such as interpolation, coordinate transformation, etc.
git clone --depth 1 https://github.com/OpenGHz/robot_tools.git
pip install ./robot_tools && rm -rf robot_tools
# install inner python packages
pip install ./src/airbot_play_follow_basic

# install the required ROS packages through rosdep
rosdep install --from-path src --ignore-src -r -y || {
	echo -e "${WARN}rosdep install failed (not complete), please check your rosdep.${DEF}"
	rosdepwarn=1
}

# OK
if [ "$rosdepwarn" = 1 ]; then
	echo -e "${OK}ROS config OK with warning.${DEF}"
else
	echo -e "${OK}Config OK.${DEF}"
fi
