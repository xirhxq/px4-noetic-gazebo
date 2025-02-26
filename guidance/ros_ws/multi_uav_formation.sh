#!/bin/bash

scene=$1
numbers=$2
leaders=$3

if [ -z "$2" ]; then
    echo "Usage: ./multi_uav_formation.sh <scene_name> <number_of_uavs> <number_of_leaders>"
    exit 1
fi

for ((i=1; i<=numbers; i++))
do
    tmux split-window -h "bash -c 'source devel/setup.bash; rosrun multi_uav_formation SingleRun.py --number $i --scene $scene; exec bash'"
done

tmux select-layout tiled
