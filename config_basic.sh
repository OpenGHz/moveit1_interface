#!/bin/bash

# set the color variables of 'echo' cmd
DEF='\e[0m'
OK='\e[1;32m'
WARN='\e[1;33m'

# install useful robot tools such as interpolation, coordinate transformation, etc.
echo "Trying to install packages from github..."
git clone --depth 1 https://github.com/OpenGHz/robot_tools.git || exit 0
pip install ./robot_tools && rm -rf robot_tools || exit 0
# install inner python packages
pip install ./src/airbot_play_follow_basic || exit 0

# install the required ROS packages through rosdep
rosdep install --from-path src --ignore-src -r -y || {
	echo -e "${WARN}rosdep install failed or not completely, please check your rosdep.${DEF}"
	rosdepwarn=1
}

# OK
if [ "$rosdepwarn" = 1 ]; then
	echo -e "${OK}ROS config OK with warning.${DEF}"
else
	echo -e "${OK}Config OK.${DEF}"
fi
