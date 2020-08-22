#!/bin/bash

image_name="airsim_controller"

docker build . \
	-t $image_name:nvidia_docker1 \
	--build-arg CACHEBUST=$(date +%s)
