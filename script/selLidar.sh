#!/bin/bash

## if no parameter, then exit after print usage
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 target"
    echo "target: select one of these"
    echo "rplidar, ydlidar"
	exit 1
fi

if [ "$1" == "rplidar" ]; then
    echo "rplidar set"
    sed -i 's/ydlidar/rplidar/g' ../monicar2_bringup/launch/bringup.launch.py
else
    echo "ydlidar set"
    sed -i 's/rplidar/ydlidar/g' ../monicar2_bringup/launch/bringup.launch.py
fi 