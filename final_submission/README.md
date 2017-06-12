# Team setup script for final run

Please read the following pages first:

[Final Submission instructions](https://bitbucket.org/osrf/ariac/wiki/finals)

[ARIAC Docker](https://github.com/osrf/ariac-docker)

## Test code with this config

### Before you start:

In **line 18** of **build_team_system.bash**:
 `git clone https://github_name:password@github.com/cwru-robotics/cwru_ariac.git`

Change `github_name` and `password` to your own github name and password.

## Getting ariac docker

Clone ariac docker from github:

```
mkdir -p ~/ariac_ws && cd ~/ariac_ws
git clone https://github.com/osrf/ariac-docker
cd ariac-docker
```

Copy team config to ariac docker:

```
roscd cwru_ariac/../final_submission
cp -r team_case ~/ariac_ws/ariac-docker/team_configs/
```

## Installing Docker

Please, follow [these instructions](https://docs.docker.com/engine/installation/linux/ubuntu/) and install `Docker CE`.

Continue to the [post-install instructions](https://docs.docker.com/engine/installation/linux/linux-postinstall/) and complete the "Manage Docker as a non-root user" section to avoid having to run the commands on this page using `sudo`.

## Fetch the ARIAC system

To prepare the ARIAC competition system (but not run it), call:

```
cd ~/ariac_ws
./pull_dockerhub_images.bash
```

## Preparing your team's system

To prepare your team's system (but not run it), call:

```
./prepare_team_system.bash team_case
```

## Running a single trial

To run a specific trial (in this case the trial called `example_trial1`), call:

```
./run_trial.bash team_case example_trial1

# To run other trials:
# ./run_trial.bash team_case <trial_name>
```

### Playing back the simulation

To play-back a specific trial's log file, you must have ARIAC installed on your machine, and then you can call:

```
roslaunch osrf_gear gear_playback.launch state_log_path:=`pwd`/logs/team_case/example_trial1/gazebo/state.log
```

## Running all trials

To run all trials listed in the `comp_configs` directory, call:

```
./run_all_trials.bash team_case
```

This will run each of the trials sequentially automatically.
This is the invocation that will be used to test submissions for the Finals: your system will not be provided with any information about the trial number or the conditions of the trial.
If your system performs correctly with this invocation, regardless of the set of configuration files in the `comp_configs` directory, you're ready for the competition.

### Stopping the competition/containers

If during your development you need to kill the ARIAC server/competitor containers, you can do so with:

```
./kill_ariac_containers.bash
```

This will kill and remove all ARIAC containers.
