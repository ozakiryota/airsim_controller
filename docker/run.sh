#!/bin/bash

image_name="airsim_controller"
root_path=$(pwd)

xhost +
docker run -it --rm \
	--env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--net=host \
	-v $root_path/../save:/home/airsim_ws/$image_name/save \
	-v $root_path/../src:/home/airsim_ws/$image_name/src \
	$image_name:docker

	# -v $root_path/../src:/home/airsim_ws/$image_name/src \
