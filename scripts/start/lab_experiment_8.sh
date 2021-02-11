#!/bin/bash

# init workspace
source ~/drone_simulator_ws/src/multi_uav_se_mission/scripts/init/init_ws.sh

# create a results folder if it not exists
mkdir -p /home/$USER/drone_simulator_ws/src/multi_uav_se_mission/bag

# launch
roslaunch multi_uav_se_mission lab_experiment_8.launch
