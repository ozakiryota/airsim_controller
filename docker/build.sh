#!/bin/bash

image_name="airsim_controller"

docker build . \
	-t $image_name:docker \
	--build-arg CACHEBUST=$(date +%s)
