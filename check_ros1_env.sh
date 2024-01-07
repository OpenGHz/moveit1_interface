#!/usr/bin/env bash

if [ -z "$1" ]; then
    ros_name="noetic"
else
    if [ "$1" = "rosdepc" ]; then
        install_rosdepc="true"
        shift
    fi
    if [ -z "$1" ]; then
        ros_name="noetic"
    else
        ros_name="$1"
    fi
fi

# get the shell type
str="${SHELL}"
shtype=${str##*/}
shell_name=${shtype}"rc"

# check whether ROS has been installed, and whether the ROS has been correctly sourced
if [ "${ROS_ROOT}" = "" ]; then
    if [ -d /opt/ros/"${ros_name}"/share/ros ]; then
        echo -e "${FAILED}Please check whether you have correctly sourced the /opt/ros/${ros_name}/setup.*sh directory in this terminal or in your .*rc file.${shell_name} ${DEF}" && exit 0
    else
        echo -e "${FAILED}Please install ROS-${ros_name} first.${DEF}" && exit 0
    fi
fi

# install catkin tools ()
sudo apt-get install python3-catkin-tools 2>/dev/null || pip3 install catkin-tools catkin-tools-python 2>/dev/null

# install rosdepc
if [ -n "${install_rosdepc}" ]; then
    sudo pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple rosdepc || { echo -e "${FAILED}Failed to install rosdepc, please try to use this cmd to install again: echo 3 | { wget http://fishros.com/install -O fishros && bash fishros; }.${DEF}" && exit 0; }
    output=$(rosdepc init 2>&1)
    if { echo "$output" | grep -q "找不到命令"; } || { echo "$output" | grep -q "command not found"; }; then
        echo -e "${FAILED}Failed to install rosdepc completely, please try to use this cmd to install again: echo 3 | { wget http://fishros.com/install -O fishros && bash fishros; }.${DEF}" && exit 0
    fi
    { { sudo rosdepc init >/dev/null 2>&1 || rosdepc init >/dev/null; } && rosdepc update; } >/dev/null || { echo -e "${FAILED}rosdepc failed to be ready.${DEF}" && exit 0; }
fi
