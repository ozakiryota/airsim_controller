#include <iostream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

class DroneWayPointFlight{
	private:
		msr::airlib::MultirotorRpcLibClient _client;
		std::vector<Eigen::Vector3f> _path;

	public:
		DroneWayPointFlight();
		void setWayPoint(void);
		void clientInitialization(void);
		void startFlight(void);
};

DroneWayPointFlight::DroneWayPointFlight()
{
	std::cout << "----- drone_waypoint_flight -----" << std::endl;
	/*initialize*/
	setWayPoint();
	clientInitialization();
}

void DroneWayPointFlight::setWayPoint(void)
{
	const double height = -3.0;
	_path = {
		Eigen::Vector3f(128.0, 0.0, height),
		Eigen::Vector3f(128.0, 128.0, height),
		Eigen::Vector3f(0.0, 128.0, height),
		Eigen::Vector3f(-128.0, 128.0, height),
		Eigen::Vector3f(-128.0, 0.0, height),
		Eigen::Vector3f(-128.0, -128.0, height),
		Eigen::Vector3f(0.0, -128.0, height),
		Eigen::Vector3f(128.0, -128.0, height),
		Eigen::Vector3f(0.0, -128.0, height),
		Eigen::Vector3f(0.0, 0.0, height)
	};
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
	const double vel = 5.0;
	msr::airlib::YawMode yaw_mode;
	yaw_mode.is_rate = false;
	yaw_mode.yaw_or_rate = 0;
	_client.moveOnPathAsync(
		_path,
		vel,
		msr::airlib::Utils::max<float>(),
		msr::airlib::DrivetrainType::ForwardOnly,
		yaw_mode
	)->waitOnLastTask();
}

int main(void) 
{
	DroneWayPointFlight drone_waypoint_flight;
	drone_waypoint_flight.startFlight();

	return 0;
}
