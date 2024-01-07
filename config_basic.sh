#!/bin/bash
set -e
# install tkinter
sudo apt-get install python3-tk
# install useful robot tools such as interpolation, coordinate transformation, etc.
echo "Trying to install packages from github..."
if [ ! -d "robot_tools" ]; then
    git clone --depth 1 https://github.com/OpenGHz/robot_tools.git
fi
pip install ./robot_tools -i https://pypi.tuna.tsinghua.edu.cn/simple
# install inner python packages
pip install ./src/airbot_play_follow_basic -i https://pypi.tuna.tsinghua.edu.cn/simple
# OK
echo -e "\033[32m[INFO] Install successfully! \033[0m"