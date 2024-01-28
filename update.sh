#!/usr/bin/env bash
set -e

DEF='\e[0m'
OK='\e[1;32m'

if [ ! -d "robot_tools" ]; then
    git clone --depth 1 https://github.com/OpenGHz/robot_tools.git
else
    cd robot_tools
    git pull
    cd ..
fi

# $1 can be -e
if [ -n "$1" ];then
    pip install "$1" ./robot_tools -i https://pypi.tuna.tsinghua.edu.cn/simple
    pip install "$1" ./src/airbot_play_follow_basic
else
    pip install ./robot_tools -i https://pypi.tuna.tsinghua.edu.cn/simple
    pip install ./src/airbot_play_follow_basic
fi

echo -e "${OK}Update OK.${DEF}"