# CWRU ARIAC

ARIAC (Agile Robotics for Industrial Automation Competition) is a competition hosted by the National Institute of Standards and Technology (NIST).

## cwru_ariac

Code for doing this competition, under development.

## osrf_gear

Official ARIAC repository

Modified from [OSRF GEAR](https://bitbucket.org/osrf/ariac/overview).

System setup see [here](https://github.com/cwru-robotics/cwru_scripts/blob/master/ariac/ariac.sh).

## gear_example

Official exmaple from OSRF

##how to start up simulation:
empty (no robot, no parts, no camera):
`rosrun osrf_gear gear.py`
NIST example:
rosrun osrf_gear gear.py -f `rospack find osrf_gear`/config/sample.yaml

with our config:
rosrun osrf_gear gear.py -f `rospack find osrf_gear`/config/qual1a.yaml `rospack find osrf_gear`/config/robot_and_sensors.yaml
