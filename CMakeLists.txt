cmake_minimum_required(VERSION 3.5.0)
project(airsim_controller)

add_compile_options(-std=c++11 -O2 -g -Wall)

find_package(OpenCV)
find_package(PCL)

## for "cnpy"
link_directories(/usr/local/lib)

# set(AIRSIM_ROOT ../AirSim)
find_path(AIRSIM_ROOT
	NAMES AirSim.sln 
	PATHS "./AirSim" "../AirSim" "../../AirSim" "../../../AirSim" "../../../../AirSim"
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
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)


add_executable(hello_drone src/hello_drone.cpp)
target_link_libraries(hello_drone AirLib rpc)
#
add_executable(drone_random_pose src/drone_random_pose.cpp)
target_link_libraries(drone_random_pose AirLib rpc ${OpenCV_LIBRARIES} cnpy)
#
add_executable(drone_random_flight src/drone_random_flight.cpp)
target_link_libraries(drone_random_flight AirLib rpc)
#
add_executable(drone_random_flight_updown src/drone_random_flight_updown.cpp)
target_link_libraries(drone_random_flight_updown AirLib rpc)
#
add_executable(drone_waypoint_flight src/drone_waypoint_flight.cpp)
target_link_libraries(drone_waypoint_flight AirLib rpc)
#
add_executable(drone_waypoint_flight_withnoise src/drone_waypoint_flight_withnoise.cpp)
target_link_libraries(drone_waypoint_flight_withnoise AirLib rpc)
#
add_executable(drone_set_pose src/drone_set_pose.cpp)
target_link_libraries(drone_set_pose AirLib rpc ${OpenCV_LIBRARIES})
#
add_executable(save_lidar_data src/save_lidar_data.cpp)
target_link_libraries(save_lidar_data AirLib rpc ${PCL_LIBRARIES} cnpy)
