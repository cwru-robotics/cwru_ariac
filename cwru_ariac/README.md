# cwru_ariac

Overall structure see here:
```
          ←←←←←←←CameraEstimator
          ↓            ↓
          ↓      ConveyorManager
          ↓            ↓
     BinManager → GlobalManager ← Parts on AGV and gound
          ↑            ↓
    PartsSorter   GlobalPlanner ← RobotPlanner
                       ↑
                  OrderManager
```
Since the logical camera is a cheat camera, only a single camera can done the job of the whole competition.

This is configurable in *config/ariac_conf.yaml*, including sensors and initial joint state.

## Modules

**AriacBase**: base of every classes (Done)

**CameraEstimator**: watching camera updates keep tracking parts (Done, Tested)

**ConveyorManager**: provide planning interface for parts on Conveyor (Done, Not tested)

**BinManager**: maintain each bin cells provide temporary location for parts

**OrderManager**: start competition, check score, and submit orders (call AGV) (Done, Not tested)

**RobotPlanner**: planning interface for UR10 + extra linear tray (7DOF)

**Cheater**: extra interface for competition control, won't available in real competition (Done, Not tested)

## Running tests/demos

###Before run: 

Run setup script from [cwru_scripts](https://github.com/cwru-robotics/cwru_scripts/blob/master/ariac/ariac.sh)

Clone this repository and do `catkin_make` and `catkin_make install`

Start simulator by one of following commands:

With all parts on Bins:
``
rosrun osrf_gear gear.py -f `rospack find osrf_gear`/config/comp_conf1.yaml `rospack find cwru_ariac`/config/ariac_conf.yaml
``

With some of parts on Bins:
``
rosrun osrf_gear gear.py -f `rospack find osrf_gear`/config/comp_conf2.yaml `rospack find cwru_ariac`/config/ariac_conf.yaml
``

With no parts on Bins:
``
rosrun osrf_gear gear.py -f `rospack find cwru_ariac`/config/ariac_conf.yaml
``

Optional args:

`-o /tmp` -- this generates temporary launch files in /tmp directory

###Executables:

`rosrun cwru_ariac pose_tunner`: a tool for change joints values arbitrary, with joint value feedback. 

`rosrun cwru_ariac pick_and_place_demo`: a dumb demo for pick and place part, will failed occasionally.

`rosrun cwru_ariac camera_tester [part_id]`: a test for keep tracking a single part, note each part have been assigned a positive integer as its own part id. 
For example, you can try `rosrun cwru_ariac camera_tester 55` or `rosrun cwru_ariac camera_tester 110 # and start the competition`
    
