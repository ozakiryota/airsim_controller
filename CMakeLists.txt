cmake_minimum_required(VERSION 3.5.0)
project(airsim_controller)

add_compile_options(-std=c++11)

# set(AIRSIM_ROOT ../AirSim)
find_path(AIRSIM_ROOT
	NAMES AirSim.sln 
	PATHS "../AirSim"
)
message(STATUS "found AIRSIM_ROOT=${AIRSIM_ROOT}")

add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

include_directories(
	${AIRSIM_ROOT}/AirLib/include
	${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include
	${AIRSIM_ROOT}/MavLinkCom/include
	${AIRSIM_ROOT}/MavLinkCom/common_utils
	${AIRSIM_ROOT}/AirLib/deps/eigen3
)

add_executable(hello_drone src/hello_drone.cpp)
target_link_libraries(hello_drone AirLib rpc)
