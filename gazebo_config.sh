#!/bin/bash

current_dir=$(pwd)
if [ -n "$GAZEBO_RESOURCE_PATH" ];then  # the GAZEBO_RESOURCE_PATH is not empty showing the gazebo has been configured
    echo "export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:${current_dir}/src/gazebo" >> ~/."${SHELL##*/}"rc
    GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:${current_dir}/src/gazebo && export GAZEBO_RESOURCE_PATH
else
    echo "export GAZEBO_RESOURCE_PATH=${current_dir}/src/gazebo" >> ~/."${SHELL##*/}"rc
    GAZEBO_RESOURCE_PATH=${current_dir}/src/gazebo && export GAZEBO_RESOURCE_PATH
    # source the gazebo setup.bash file
    echo 'current_dir=$(pwd) && cd /usr/share/gazebo-*/ && source setup.bash && cd ${current_dir} && unset current_dir' >> ~/."${SHELL##*/}"rc
    cd /usr/share/gazebo-*/ && source setup.bash && cd "$current_dir"
fi
# echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${current_dir}/src/gazebo/models" >> ~/."${SHELL##*/}"rc

# OK
echo -e "\e[1;32mGazebo config OK.\e[0m"