#!/bin/bash

cd ~/ros2_ws
colcon build --packages-select livox_lio
source ./install/setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
ros2 launch lio_livox mid360_lio_launch.py
