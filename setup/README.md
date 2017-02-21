# Setup ARIAC

## System Setup

### Fresh install

If you have a newly installed Ubuntu 14.04 without ROS installed:

```
cd [directory where this file locates]

chmod +x install_ros_ariac.sh setup_workspace.sh

./install_ros_ariac.sh
```

Enter your password and wait...

If you want to setup workspace also, run:

`./setup_workspace.sh`

### Already have ros indigo installed

Then you have to uninstall gazebo2 first and then install gazebo7:

```
sudo apt-get --yes --force-yes remove gazebo2 ros-indigo-gazebo-*
sudo apt-get --yes --force-yes install gazebo7 gazebo7-plugin-base gazebo7-common libgazebo7
sudo apt-get --yes --force-yes install ros-indigo-gazebo7-ros-*
```

## Run ARIAC

### Requirements

Clone this repository in your catkin workspace under *src* folder and do

```
cd [directory to your catkin workspace]

catkin_make

catkin_make install
```

### Start the simulation

To start a empty world (no robot, no parts, no camera):
```
`rosrun osrf_gear gear.py`
```

To start qualifiers 1A:
```
rosrun osrf_gear gear.py -f `rospack find osrf_gear`/config/qual1a.yaml `rospack find cwru_ariac`/config/qual1.yaml
```

To start qualifiers 1B:
```
rosrun osrf_gear gear.py -f `rospack find osrf_gear`/config/qual1b.yaml `rospack find cwru_ariac`/config/qual1.yaml
```

To start a world with full bins:
```
rosrun osrf_gear gear.py -f `rospack find osrf_gear`/config/comp_conf1.yaml `rospack find cwru_ariac`/config/ariac_conf.yaml
```

There is two more example configure:
```
rosrun osrf_gear gear.py -f `rospack find osrf_gear`/config/comp_conf2.yaml `rospack find cwru_ariac`/config/ariac_conf.yaml

rosrun osrf_gear gear.py -f `rospack find osrf_gear`/config/sample.yaml `rospack find cwru_ariac`/config/ariac_conf.yaml
```

### Using moveit

To use moveit to control the robot, run following commands:
```
roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch sim:=true

roslaunch ur10_moveit_config moveit_rviz.launch config:=true
```
