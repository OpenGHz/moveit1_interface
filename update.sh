#!/bin/bash

DEF='\e[0m'
OK='\e[1;32m'

if [ "$1" = '-e' ];then
    pip install -e ./src/airbot_play_follow_basic
else
    pip install ./src/airbot_play_follow_basic
fi

echo -e "${OK}Update OK.${DEF}"