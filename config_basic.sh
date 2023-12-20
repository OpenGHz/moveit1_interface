#!/bin/bash

# install useful robot tools such as interpolation, coordinate transformation, etc.
echo "Trying to install packages from github..."
git clone --depth 1 https://github.com/OpenGHz/robot_tools.git || exit 0
pip install ./robot_tools && rm -rf robot_tools || exit 0
# install inner python packages
pip install ./src/airbot_play_follow_basic || exit 0