# CWRU ARIAC

ARIAC (Agile Robotics for Industrial Automation Competition) is a competition hosted by the National Institute of Standards and Technology (NIST).

**TeamCase**

## The cwru_ariac Package

Core code for complete ARIAC competition see [cwru_ariac Readme](https://github.com/cwru-robotics/cwru_ariac/blob/master/cwru_ariac/README.md)

### For Final Competition:

- Start ariac environment with any command from the following qualifiers or from setup

``rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/qual3b.yaml `rospack find cwru_ariac`/config/final.yaml``

- Start robot move action server:

`rosrun robot_move_as robot_move_as`

- Start the laser speedometer:

`rosrun cwru_ariac laser_speedometer_bae`

- Run the competition code:

`rosrun cwru_ariac final`
`rosrun cwru_ariac competition_main`

- (Alternative) start by launch file:

`roslaunch cwru_ariac run_comp.launch`

### Qualifier 1:

For qual1a:

``rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/qual1a.yaml `rospack find cwru_ariac`/config/qual1.yaml``

OR, for qual1b:

``rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/qual1b.yaml `rospack find cwru_ariac`/config/qual1.yaml``

Then run:

`rosrun robot_move_as robot_move_as`

`rosrun cwru_ariac qualifier1`

### Qualifier 2:

For qual2a:

``rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/qual2a.yaml `rospack find cwru_ariac`/config/qual2.yaml``

OR, for qual2b:

``rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/qual2b.yaml `rospack find cwru_ariac`/config/qual2.yaml``

Then run:

`rosrun robot_move_as robot_move_as`

`rosrun cwru_ariac qualifier2`

### Qualifier 3:

For qual3a:

``rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/qual3a.yaml `rospack find cwru_ariac`/config/qual3.yaml``

OR, for qual3b:

``rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/qual3b.yaml `rospack find cwru_ariac`/config/qual3.yaml``

Then run:

`rosrun robot_move_as robot_move_as`

`rosrun cwru_ariac qualifier3`

## How to start up the simulation:

See: [Setup](https://github.com/cwru-robotics/cwru_ariac/tree/master/setup)

## Update ARIAC

The lastest ARIAC version as of 6/11/17 is 1.1.4
We will always keep using the lastest ARIAC version.

To check your current ARIAC version, run following command:

`dpkg -s ariac|grep Version`

To update your ARIAC, run following commands:

```
sudo apt-get update
sudo apt-get upgrade ariac
```
