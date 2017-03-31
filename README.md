# CWRU ARIAC

ARIAC (Agile Robotics for Industrial Automation Competition) is a competition hosted by the National Institute of Standards and Technology (NIST).

## cwru_ariac

TO RUN THE QUALIFIER CODE:

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

## osrf_gear

Official ARIAC repository

Modified from [OSRF GEAR](https://bitbucket.org/osrf/ariac/overview).

## gear_example

Official exmaple from OSRF

## How to start up simulation:

See: [Setup](https://github.com/cwru-robotics/cwru_ariac/tree/master/setup)

## Update ARIAC

The lastest ARIAC version is 1.0.4, we will always keep using the lastest ARIAC version.

To check your current ARIAC version, run following command:

`dpkg -s ariac|grep Version`

To update your ARIAC, run following commands:

```
sudo apt-get update 
sudo apt-get upgrade ariac
```

To check what version of ariac you are running, do:
`dpkg -s ariac | grep Version`

