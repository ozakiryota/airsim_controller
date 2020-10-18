FROM osrf/ros:kinetic-desktop-full

########## Basis ##########
RUN apt-get update && apt-get install -y \
	vim \
	wget \
	unzip \
	git \
	build-essential
########## AirSim ##########
# RUN apt-get update &&\
# 	apt-get install -y \
# 		rsync \
# 		g++-8 \
# 		python-catkin-tools \
# 		ros-kinetic-mavros* &&\
RUN mkdir /home/airsim_ws &&\
	cd /home/airsim_ws &&\
	git clone https://github.com/Microsoft/AirSim.git &&\
	cd AirSim &&\
	./setup.sh &&\
	./build.sh
######### AirSim controler ##########
##### CMake >= 3.10 #####
RUN mkdir /home/cmake_ws &&\
	cd /home/cmake_ws &&\
	wget https://github.com/Kitware/CMake/releases/download/v3.17.3/cmake-3.17.3.tar.gz &&\
	tar xvf cmake-3.17.3.tar.gz &&\
	cd cmake-3.17.3 &&\
	./bootstrap && \
	make -j $(nproc --all) &&\
	make install
###### OpenCV 2.4.9 #####
## https://opencv.org/releases/
RUN apt-get update && apt-get install -y qtbase5-dev &&\
	mkdir -p /home/opencv_ws &&\
	cd /home/opencv_ws &&\
	wget https://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.4.9/opencv-2.4.9.zip/download &&\
	unzip download &&\
	cd opencv-2.4.9 &&\
	mkdir build &&\
	cd build &&\
	cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON .. &&\
	make -j $(nproc --all) &&\
	make install
##### Avoiding error #####
RUN sed -i "s/#include <filesystem>/#include <experimental\/filesystem>/g" /home/airsim_ws/AirSim/MavLinkCom/MavLinkTest/Commands.cpp &&\
	sed -i "s/using namespace std::filesystem;/using namespace std::experimental::filesystem;/g" /home/airsim_ws/AirSim/MavLinkCom/MavLinkTest/Commands.cpp &&\
	sed -i "s/#include <filesystem>/#include <experimental\/filesystem>/g" /home/airsim_ws/AirSim/MavLinkCom/MavLinkTest/main.cpp &&\
	sed -i "s/using namespace std::filesystem;/using namespace std::experimental::filesystem;/g" /home/airsim_ws/AirSim/MavLinkCom/MavLinkTest/main.cpp
##### NO cache #####
ARG CACHEBUST=1
##### My project #####
RUN cd /home/airsim_ws &&\
	git clone https://github.com/ozakiryota/airsim_controller &&\
	cd airsim_controller &&\
	mkdir build &&\
	cd build &&\
	cmake .. &&\
	make -j $(nproc --all)
######### initial position ##########
WORKDIR /home/airsim_ws/airsim_controller/build
