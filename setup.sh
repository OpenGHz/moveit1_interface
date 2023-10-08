#!/bin/bash

# export ros ws path
source ./devel/setup."${SHELL##*/}" || { echo -e "\e[1;31mPlease use 'source' cmd to run this setup.sh file!\e[0m" && exit 0; }

# OK
echo -e "\e[1;32mOK!\e[0m"