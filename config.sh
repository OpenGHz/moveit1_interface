#!/bin/bash

# set the color variables of 'echo' cmd
DEF='\e[0m'
OK='\e[1;32m'
FAILED='\e[1;31m'
WARN='\e[1;33m'

# get the shell type
str="${SHELL}";shtype=${str##*/};name=${shtype}"rc";

# check whether ROS has been installed, and whether the ROS has been correctly sourced
if [ "${ROS_ROOT}" == "" ];then
	if [ -d /opt/ros/noetic/share/ros ];then
		echo -e "${FAILED}Please check whether you have correctly sourced the /opt/ros/noetic/setup.*sh directory in this terminal or in your .*rc file.${name} ${DEF}" && exit 0
	else echo -e "${FAILED}Please install ROS-Noetic first.${DEF}" && exit 0
	fi
fi

# install useful robot tools such as interpolation, coordinate transformation, etc.
git clone --depth 1 https://github.com/OpenGHz/robot_tools.git
pip install ./robot_tools && rm -rf robot_tools
# install the required python packages
pip3 install scipy || { echo -e "${FAILED}scipy install failed.${DEF}" && exit 0; }
# install inner python packages
pip install ./src/airbot_play_follow_basic

# install the required ROS packages through rosdep
sudo pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple rosdepc || { echo -e "${FAILED}Failed to install rosdepc, please try to use this cmd to install again: echo 3 | { wget http://fishros.com/install -O fishros && bash fishros; }.${DEF}" && exit 0; }
output=$(rosdepc init 2>&1)
if { echo "$output" | grep -q "找不到命令"; } || { echo "$output" | grep -q "command not found"; }; then
    echo -e "${FAILED}Failed to install rosdepc completely, please try to use this cmd to install again: echo 3 | { wget http://fishros.com/install -O fishros && bash fishros; }.${DEF}" && exit 0
fi
{ { sudo rosdepc init > /dev/null 2>&1 || rosdepc init > /dev/null; } && rosdepc update; } > /dev/null || { echo -e "${FAILED}rosdepc failed to be ready.${DEF}" && exit 0; }
rosdep install --from-path src --ignore-src -r -y || { echo -e "${WARN}rosdep install failed(not complete), please check your rosdep.${DEF}";rosdepwarn=1; }

# install catkin tools
sudo apt-get install python3-catkin-tools 2> /dev/null || pip3 install catkin-tools catkin-tools-python 2> /dev/null

# OK
if [ "$rosdepwarn" == 1 ];then echo -e "${OK}ROS config OK with warning.${DEF}"
else echo -e "${OK}Config OK.${DEF}"
fi
