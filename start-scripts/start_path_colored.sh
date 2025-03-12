#!/bin/bash

cd ~/ros2_ws
colcon build --packages-select col_pth
source ./install/setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
ros2 launch col_pth col_pth_launch.py
