#!/bin/bash

cd ~/ros2_ws
colcon build --packages-select slam_gmapping 
source ./install/setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
ros2 launch slam_gmapping slam_gmapping.launch.py
