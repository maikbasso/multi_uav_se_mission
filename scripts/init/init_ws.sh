#!/bin/bash

# go to workspace
cd /home/$USER/drone_simulator_ws/

# ROS
source devel/setup.bash

# PX4
px4_dir=/home/$USER/drone_simulator_ws/multi_uav_dependencies/firmware/px4
#source $px4_dir/Tools/setup_gazebo.bash $px4_dir $px4_dir/build/posix_sitl_default
#export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir
#export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir/Tools/sitl_gazebo
source $px4_dir/Tools/setup_gazebo.bash $px4_dir $px4_dir/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir/Tools/sitl_gazebo
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/$USER/drone_simulator_ws/devel/lib
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/$USER/drone_simulator_ws/src/multi_uav_se_mission/models

# go to workspace
cd /home/$USER/drone_simulator_ws/
