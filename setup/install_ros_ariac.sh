#!/bin/bash

echo -e "\e[1m \e[34m >>> Beginning ROS Indigo Installation \e[21m \e[39m"

echo -e "\e[34m >>> Setting up sources.list and keys... \e[39m"

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

echo -e "\e[34m >>> ...done\e[39m"

sudo apt-get update

echo -e "\e[34m >>> Install Gazebo7 \e[39m"

sudo apt-get --yes --force-yes install gazebo7 gazebo7-plugin-base gazebo7-common libgazebo7

echo -e "\e[34m >>> Beginning ros-indigo-desktop-full installation...\e[39m"

sudo apt-get --yes --force-yes install ros-indigo-desktop-full

echo -e "\e[34m >>> Setting up rosdep\e[39m"

sudo rosdep init
rosdep update

echo -e "\e[34m >>> Setting up environment \e[39m"

cat bashrc >> ~/.bashrc
source ~/.bashrc

echo -e "\e[1m \e[34m >>> Installing dependencies \e[21m \e[39m"

sudo apt-get --yes --force-yes install python-rosinstall
sudo apt-get --yes --force-yes install ros-indigo-gazebo7-ros-*
sudo apt-get --yes --force-yes install ros-indigo-joy ros-indigo-costmap* ros-indigo-nav-core ros-indigo-sound-play ros-indigo-amcl ros-indigo-move-base ros-indigo-joint-state-controller ros-indigo-effort-controllers ros-indigo-stdr-simulator
sudo apt-get --yes --force-yes install ros-indigo-navigation ros-indigo-csm ros-indigo-laser-geometry
sudo apt-get --yes --force-yes install ros-indigo-control* ros-indigo-robot-controllers* ros-indigo-robot-state-publisher
sudo apt-get --yes --force-yes install ros-indigo-ros-cont* ros-indigo-joint-*
sudo apt-get --yes --force-yes install ros-indigo-rqt ros-indigo-rqt*
sudo apt-get --yes --force-yes install ros-indigo-moveit ros-indigo-moveit-core ros-indigo-moveit-kinematics ros-indigo-moveit-ros-planning ros-indigo-moveit-ros-move-group ros-indigo-moveit-planners-ompl ros-indigo-moveit-ros-visualization ros-indigo-moveit-simple-controller-manager
sudo apt-get --yes --force-yes install ros-indigo-object-recognition-*
sudo apt-get --yes --force-yes install ariac

echo -e "\e[1m \e[34m >>> Installing support software \e[21m \e[39m"
sudo apt-get --yes --force-yes install git build-essential
sudo apt-get --yes --force-yes install python-pip python-dev build-essential
sudo pip install --upgrade pip
sudo pip install --upgrade virtualenv
sudo pip install numpy scipy
sudo pip install virtualenv virtualenvwrapper

sudo add-apt-repository -y ppa:webupd8team/sublime-text-3
sudo apt-get update
sudo apt-get --yes --force-yes install sublime-text-installer atom
sudo apt-get --yes --force-yes install kazam vlc
sudo apt-get --yes --force-yes install gitk git-gui

echo -e "\e[34m Setup complete! \e[39m"
