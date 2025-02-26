#!/bin/bash

IMAGE_NAME="xirhxq/px4-noetic-gazebo"
TAG="latest"
CONTAINER_NAME="px4_noetic_dev"
USER="tii_dev"

script_dir=$(cd "$(dirname "$0")" && pwd)
ros_ws=$script_dir/../guidance/ros_ws
echo "ROS workspace: $ros_ws"

XAUTH=/tmp/.docker.xauth
xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    touch $XAUTH
    chmod a+r $XAUTH
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    fi
fi

if [ ! -f $XAUTH ]; then
  exit 1
fi

xhost +local:root > /dev/null

docker run -it --rm \
    --privileged \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --network host \
    --workdir="/app/guidance/ros_ws" \
    --volume="$ros_ws/..:/app/guidance:rw" \
    --volume="/dev:/dev" \
    --volume="/etc/localtime:/etc/localtime:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --name=${CONTAINER_NAME} \
    --user=${USER} \
    $IMAGE_NAME:${TAG} \
    "$@"

xhost -local:root > /dev/null
