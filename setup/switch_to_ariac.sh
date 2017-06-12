#!/bin/bash

echo -e "\e[34m >>> Setting up sources.list and keys... \e[39m"

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt-get update

echo -e "\e[34m >>> Install Gazebo7 \e[39m"

sudo apt-get --yes --force-yes remove gazebo2 ros-indigo-gazebo-*
sudo apt-get --yes --force-yes install gazebo7 gazebo7-plugin-base gazebo7-common libgazebo7
sudo apt-get --yes --force-yes install ros-indigo-gazebo7-ros-*

cat bashrc >> ~/.bashrc
source ~/.bashrc

echo -e "\e[1m \e[34m >>> Installing dependencies \e[21m \e[39m"

sudo apt-get --yes --force-yes install python-rosinstall
sudo apt-get --yes --force-yes install ros-indigo-joint-state-controller ros-indigo-effort-controllers ros-indigo-stdr-simulator
sudo apt-get --yes --force-yes install ros-indigo-navigation ros-indigo-csm ros-indigo-laser-geometry
sudo apt-get --yes --force-yes install ros-indigo-control* ros-indigo-robot-controllers* ros-indigo-robot-state-publisher
sudo apt-get --yes --force-yes install ros-indigo-ros-cont* ros-indigo-joint-*
sudo apt-get --yes --force-yes install ros-indigo-rqt ros-indigo-rqt*
sudo apt-get --yes --force-yes install ros-indigo-moveit ros-indigo-moveit-core ros-indigo-moveit-kinematics ros-indigo-moveit-ros-planning ros-indigo-moveit-ros-move-group ros-indigo-moveit-planners-ompl ros-indigo-moveit-ros-visualization ros-indigo-moveit-simple-controller-manager
sudo apt-get --yes --force-yes install ros-indigo-object-recognition-*
sudo apt-get --yes --force-yes install ariac

echo -e "\e[34m Setup complete! \e[39m"
