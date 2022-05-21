#!/bin/bash

xhost +

image="airsim_controller"
tag="latest"

docker run -it --rm \
	-e "DISPLAY" \
	-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	-v $(pwd)/../src:/root/$image/src \
	-v $HOME/rosbag:/root/rosbag \
	$image:$tag