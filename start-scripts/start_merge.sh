#!/bin/bash

cd ~/ros2_ws
colcon build --packages-select pc_mrg_flt
source ./install/setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
ros2 launch pc_mrg_flt pc_mrg_flt_launch.py
