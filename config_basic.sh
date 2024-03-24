#!/usr/bin/env bash
set -e
# install tkinter
sudo apt-get install python3-tk -y
# install useful robot tools such as interpolation, coordinate transformation, etc.
pip install robotics_tools
# install inner python packages
pip install ./src/airbot_play_ik_service
pip install ./src/airbot_play_follow_basic -i https://pypi.tuna.tsinghua.edu.cn/simple
# OK
echo -e "\033[32m[INFO] Install successfully! \033[0m"