#!/bin/bash

# prepare git pip
git --version || sudo apt update && sudo apt install git -y || exit 1
pip -V || sudo apt update && sudo apt install python3-pip -y || exit 1

if [ "$1" = "rosdepc" ];then
    # prepare rosdep
    sudo apt update && sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y || exit 1
    sudo rosdep init || exit 1
    rosdep update || exit 1
fi