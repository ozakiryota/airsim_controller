#!/bin/bash

image_name="airsim_controller"

xhost +
nvidia-docker run -it --rm \
	--env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--net=host \
	-v /home/amsl/ozaki/airsim_ws/pkgs/airsim_controller/save:/home/airsim_ws/airsim_controller/save \
	$image_name:latest
