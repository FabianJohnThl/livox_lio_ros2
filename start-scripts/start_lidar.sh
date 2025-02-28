#!/bin/bash

cd /home/fiveg/ros2_ws
colcon build --packages-select livox_ros_driver2 
source ./install/setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
ros2 launch livox_ros_driver2 MID360_LIO_launch.py
