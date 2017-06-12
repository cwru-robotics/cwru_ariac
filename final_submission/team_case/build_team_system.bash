#!/usr/bin/env bash

echo "Preparing environmen for running cwru_ariac"

# Prepare ROS
. /opt/ros/${ROS_DISTRO}/setup.bash

# Install the necessary dependencies for getting the team's source code
# Note: there is no need to use `sudo`.
apt-get update
apt-get install -y git ros-indigo-trac-ik*

# Create a catkin workspace
mkdir -p ~/ariac_ws/src
cd ~/ariac_ws/src

# Fetch the source code for our team's code
git clone https://github.com/cwru-robotics/cwru_ariac.git
git clone https://github.com/catkin/catkin_simple.git

cd ~/ariac_ws
catkin_make
