# cwru_ariac

Overall structure see here:
```
          ←←←←←←←CameraEstimator
          ↓            ↓
          ↓      SensorManager ← LaserScanner
          ↓            ↓
     BinManager → PlanningUtils
          ↑     ↗      ↓
  OraclePlanner → main function ← RobotMove ← RobotInterface
                       ↑
                  OrderManager
```

Sensors are configurable in *config/final.yaml*. For instructions, see: https://bitbucket.org/osrf/ariac/wiki/configuration_spec

## Modules

**AriacBase**: The base of every class (Done)

**CameraEstimator**: watching camera updates keep tracking parts, multi-instance of camera is allowed (Tested)

**SensorManager**: Provides interface for both camera and laser scanner, and combine information from different sources (Tested)

**BinManager**: Maintain each bin cells and provide temporary location for parts, can hold its camera and use camera information for the update.

**OrderManager**: start competition, check score, and submit orders (call AGV) (Tested)

**OraclePlanner**: quick time estimation interface for UR10 + extra linear tray (7DOF)

**RobotMove**: planning + move robot interface for UR10 + extra linear tray (7DOF) (Tested)

**RobotInterface**: low level robot interface for UR10 + extra linear tray (7DOF) (Tested)

**PlanningUtils**: Provides some planning algorithms (Tested)

**Cheater**: extra interface for competition control, won't available in real competition (Done)

## Running tests/demos

### Before run: 

Run setup script from [Setup](https://github.com/cwru-robotics/cwru_ariac/tree/master/setup)

Clone this repository and do `catkin_make` and `catkin_make install`

Start simulator:

With all parts on Bins:
``
rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/comp_conf1.yaml `rospack find cwru_ariac`/config/ariac_conf.yaml
``

Other startup command see [Setup](https://github.com/cwru-robotics/cwru_ariac/tree/master/setup)

Optional args:

`-o /tmp` -- this generates temporary launch files in /tmp directory

### xecutables:

`rosrun cwru_ariac pose_tunner`: a tool for change joints values arbitrary, with joint value feedback. 

`rosrun cwru_ariac pick_and_place_demo`: a dumb demo for pick and place part, will failed occasionally.

`rosrun cwru_ariac camera_tester [part_id]`: a test for keep tracking a single part, note each part have been assigned a positive integer as its own part id. 
For example, you can try `rosrun cwru_ariac camera_tester 55` or `rosrun cwru_ariac camera_tester 110 # and start the competition`

`rosrun cwru_ariac robot_move_as_tester`: a tester for robot_move_as in async mode

`rosrun cwru_ariac qualifier1`: qualifier 1a and 1b executable 

`rosrun cwru_ariac qualifier2`: qualifier 2a and 2b executable 

