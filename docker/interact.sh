#!/bin/bash

xhost +

image="airsim_controller"
tag="latest"

docker run -it --rm \
	-e "DISPLAY" \
	-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--net=host \
	-v $(pwd)/../save:/root/$image/save \
	-v $(pwd)/../config:/root/$image/config \
	-v $(pwd)/../src:/root/$image/src \
	-v $(pwd)/../CMakeLists.txt:/root/$image/CMakeLists.txt \
	-v $HOME/rosbag:/root/rosbag \
	$image:$tag