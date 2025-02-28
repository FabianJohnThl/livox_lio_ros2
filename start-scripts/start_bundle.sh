#!/bin/bash

cd ~

screen -XS lio quit
screen -XS lidar quit

sleep 1

screen -dmS lidar bash ./start_lidar.sh
screen -dmS lio bash ./start_lio.sh

sleep 2

screen -r
