#!/bin/bash

cd ~

screen -XS lio quit
screen -XS lidar quit
screen -XS slam quit
screen -XS pc2ls quit
screen -XS merge quit
screen -XS path_colored quit

sleep 1

screen -dmS lidar bash ./start_lidar.sh
screen -dmS lio bash ./start_lio.sh
screen -dmS slam bash ./start_slam.sh
screen -dmS pc2ls bash ./start_pc2ls.sh
screen -dmS merge bash ./start_merge.sh
screen -dmS path_colored bash ./start_path_colored.sh

sleep 2

screen -r
