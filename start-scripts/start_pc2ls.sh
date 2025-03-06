#!/bin/bash

cd ~/ros2_ws
source ./install/setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
ros2 launch ./pc2ls_launch.py
