########## Pull ##########
FROM ubuntu:20.04
########## Non-interactive ##########
ENV DEBIAN_FRONTEND=noninteractive
########## Common tool ##########
RUN apt-get update && \
	apt-get install -y \
		vim \
		wget \
		unzip \
		git \
        python-tk \
		build-essential
########## AirSim ##########
RUN cd ~/ && \
	git clone https://github.com/Microsoft/AirSim.git && \
	cd AirSim && \
	sed -i -e 's/sudo//g' setup.sh && \
	./setup.sh && \
	./build.sh
########## airsim_controller ##########
## dependencies-cnpy
RUN apt-get update && \
    apt-get install -y zlib1g-dev && \
	cd ~/ && \
	git clone https://github.com/rogersce/cnpy && \
	mkdir -p cnpy/build && \
	cd cnpy/build && \
	cmake .. && \
	make -j $(nproc --all) && \
	make install
## build
RUN apt-get update && \
    apt-get install -y \
		libopencv-dev \
		libpcl-dev && \
	cd ~/ && \
	git clone https://github.com/ozakiryota/airsim_controller.git && \
	mkdir airsim_controller/build && \
	cd airsim_controller/build
	cmake .. && \
	make -j $(nproc --all)
########## Initial position ##########
WORKDIR /root