#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: ./get_params.sh <scene_name>"
    exit 1
fi

scene=$1
echo "Getting params for $scene"
sceneconfig_path="$PWD/src/multi_uav_formation/scenes/$1.json"

key1=$(python3 -c "import json; data = json.load(open('$sceneconfig_path')); print(data['number'])")
key2=$(python3 -c "import json; data = json.load(open('$sceneconfig_path')); print(data['leaders'])")

export UAV_NUM=$key1
export LEADERS_NUM=$key2

echo "UAV_NUM: $UAV_NUM"
echo "LEADERS_NUM: $LEADERS_NUM"
