#!/bin/bash

# Bash script to source the ros and cerbrus workspaces and run the launch files
source /opt/ros/jazzy/setup.bash
source ./cerbrus_ws/install/setup.bash

echo > run_log.txt

ros2 launch cerbrus_bringup cerbrus.launch.py >>run_log.txt



