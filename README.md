# CWRU ARIAC

ARIAC (Agile Robotics for Industrial Automation Competition) is a competition hosted by the National Institute of Standards and Technology (NIST).

## cwru_ariac

TO RUN THE QUALIFIER CODE:

For qual1a:

``rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/qual1a.yaml `rospack find cwru_ariac`/config/qual1.yaml``

OR, for qual1b:

``rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/qual1b.yaml `rospack find cwru_ariac`/config/qual1.yaml``

Then run:

`rosrun robot_move_as robot_move_as`

`rosrun cwru_ariac qualifier1`

## osrf_gear

Official ARIAC repository

Modified from [OSRF GEAR](https://bitbucket.org/osrf/ariac/overview).

## gear_example

Official exmaple from OSRF

## How to start up simulation:

See: [Setup](https://github.com/cwru-robotics/cwru_ariac/tree/master/setup)
