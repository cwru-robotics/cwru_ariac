#!/bin/bash

echo -e "\e[1m \e[34m Please run install_ros_ariac before running this \e[21m \e[39m"

mkdir -p ~/ros_ws/src

cd ~/ros_ws/src  && catkin_init_workspace
cd ~/ros_ws && catkin_make

cd ~/ros_ws/src && git clone https://github.com/cwru-robotics/cwru_ariac.git

cd ~/ros_ws/src && git clone https://github.com/wsnewman/learning_ros_external_packages.git
cd ~/ros_ws/src && git clone https://github.com/wsnewman/learning_ros.git

cd ~/ros_ws && catkin_make
cd ~/ros_ws && catkin_make install

echo -e "\e[34m Setup complete, still need to run following commands: \e[39m"
echo 'git config --global user.name "your github uer name"'
echo 'git config --global user.email "your password"'
