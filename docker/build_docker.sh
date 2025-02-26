#!/bin/bash

TIMESTAMP=$(date +'%Y%m%d%H%M%S')
IMAGE_TAG=xirhxq/px4-noetic-gazebo:$TIMESTAMP
LATEST_TAG=xirhxq/px4-noetic-gazebo:latest

docker buildx create --name mybuilder --use
docker buildx inspect mybuilder --bootstrap

docker buildx build --platform linux/amd64 -t $IMAGE_TAG -t $LATEST_TAG --push .
echo "Pushed amd64 image with tags $IMAGE_TAG and $LATEST_TAG."

docker buildx build --platform linux/arm64 -t $IMAGE_TAG -t $LATEST_TAG --push .
echo "Pushed arm64 image with tags $IMAGE_TAG and $LATEST_TAG."
