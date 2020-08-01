#include <iostream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

class DroneWayPointFlight{
	private:
		msr::airlib::MultirotorRpcLibClient _client;
		std::vector<Eigen::Vector3f> _waypoints;
		std::vector<Eigen::Vector3f> _path;
		double _height = -3.0;
		double _path_resolution = 4.0;
		double _noise = 1.0;
		double _velocity = 15.0;

	public:
		DroneWayPointFlight();
		void setWayPoints(void);
		void devidePath(void);
		void addNoise(void);
		void printPath(void);
		void clientInitialization(void);
		void startFlight(void);
};

DroneWayPointFlight::DroneWayPointFlight()
{
	std::cout << "----- drone_waypoint_flight_withnoise -----" << std::endl;
	/*initialize*/
	setWayPoints();
	devidePath();
	addNoise();
	printPath();
	clientInitialization();
}

void DroneWayPointFlight::setWayPoints(void)
{
	_waypoints = {
		Eigen::Vector3f(0.0, 0.0, _height),
		Eigen::Vector3f(128.0, 0.0, _height),
		Eigen::Vector3f(128.0, 128.0, _height),
		Eigen::Vector3f(0.0, 128.0, _height),
		Eigen::Vector3f(-128.0, 128.0, _height),
		Eigen::Vector3f(-128.0, 0.0, _height),
		Eigen::Vector3f(-128.0, -128.0, _height),
		Eigen::Vector3f(0.0, -128.0, _height),
		Eigen::Vector3f(128.0, -128.0, _height),
		Eigen::Vector3f(128.0, 0.0, _height),
		Eigen::Vector3f(0.0, 0.0, _height)
	};
}

void DroneWayPointFlight::devidePath(void)
{
	for(size_t i=0; i<_waypoints.size()-1; ++i){
		Eigen::Vector3f delta = _waypoints[i+1] - _waypoints[i];
		int points_per_line = int(delta.norm()/_path_resolution);
		if(!(points_per_line > 0))	continue;
		Eigen::Vector3f step = delta/(double)points_per_line;
		for(size_t j=1; j<points_per_line; ++j){
			Eigen::Vector3f point = _waypoints[i] + j*step;
			_path.push_back(point);
		}
		_path.push_back(_waypoints[i+1]);
	}
}

void DroneWayPointFlight::addNoise(void)
{
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<> urd(-_noise, _noise);
	for(size_t i=0; i<_path.size(); ++i){
		_path[i](0) += urd(mt);
		_path[i](1) += urd(mt);
		_path[i](2) += urd(mt);
	}
}

void DroneWayPointFlight::printPath(void)
{
	for(size_t i=0; i<_path.size(); ++i){
		std::cout << i << ": ("
			<< _path[i](0) << ", "
			<< _path[i](1) << ", "
			<< _path[i](2) << ")"
		<< std::endl;
	}
	std::cout << "_path.size() = " << _path.size() << std::endl;
}

void DroneWayPointFlight::clientInitialization(void)
{
	/*connect*/
	_client.confirmConnection();
	/*reset*/
	std::cout << "Reset" << std::endl;
	_client.reset();
	/*control*/
	std::cout << "Enable API control" << std::endl;
	_client.enableApiControl(true);
	std::cout << "Arm the drone" << std::endl;
	_client.armDisarm(true);
	std::cout << "Take off" << std::endl;
	_client.takeoffAsync()->waitOnLastTask();
}

void DroneWayPointFlight::startFlight(void)
{
	std::cout << "startFlight" << std::endl;

	YawMode yaw_mode;
	yaw_mode.is_rate = false;
	yaw_mode.yaw_or_rate = 0.0;

	/* _client.moveOnPathAsync( */
	/* 	_path, */
	/* 	_velocity, */
	/* 	Utils::max<float>(), */
	/* 	DrivetrainType::ForwardOnly, */
	/* 	yaw_mode */
	/* )->waitOnLastTask(); */
	_client.moveOnPathAsync(
		_path,
		_velocity,
		Utils::max<float>(),
		DrivetrainType::ForwardOnly,
		yaw_mode,
		-1,
		0
	)->waitOnLastTask();

	std::cout << "Go home" << std::endl;
	_client.goHomeAsync()->waitOnLastTask();
}

int main(void) 
{
	DroneWayPointFlight drone_waypoint_flight;
	drone_waypoint_flight.startFlight();

	return 0;
}
