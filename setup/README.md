# System Setup

## Install ROS Kinetic

### Fresh installation

If you have not install ROS yet:

```
cd [directory of this file]

./install_ros_kinetic.sh
```

Enter your password and wait...

Further, if you want to setup your workspace, run:

`./setup_workspace.sh`

this will create a folder called **ros_ws** under your home folder, and it will becomes your ROS workspace.

## Run ARIAC

### Clone repository

**You can skip this step if you have already run setup_workspace.sh**

Clone this repository in your catkin workspace under *src* folder and do

```
ckm # An alias I made for catkin_make
```

### Start the simulation

To start a empty world (no robot, no parts, no camera):
```
rosrun osrf_gear gear.py
```

To start qualifiers 1A:
```
rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/qual1a.yaml `rospack find cwru_ariac`/config/qual1.yaml
```

To start qualifiers 1B:
```
rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/qual1b.yaml `rospack find cwru_ariac`/config/qual1.yaml
```

To start a world with full bins:
```
rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/comp_conf1.yaml `rospack find cwru_ariac`/config/ariac_conf.yaml
```

There is two more example configure:
```
rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/comp_conf2.yaml `rospack find cwru_ariac`/config/ariac_conf.yaml

rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/sample.yaml `rospack find cwru_ariac`/config/ariac_conf.yaml
```

### Using moveit

To use moveit to control the robot, run following commands:
```
roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch sim:=true

roslaunch ur10_moveit_config moveit_rviz.launch config:=true
```
