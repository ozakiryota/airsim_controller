#include <iostream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

class DroneRandomFlight{
	private:
		msr::airlib::MultirotorRpcLibClient _client;

	public:
		DroneRandomFlight();
		void clientInitialization(void);
		void updateState(void);
};

DroneRandomFlight::DroneRandomFlight()
{
	std::cout << "----- drone_waypoint_flight -----" << std::endl;
	/*initialize*/
	clientInitialization();
	updateState();
}

void DroneRandomFlight::clientInitialization(void)
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
	printState();
	std::cout << "Take off" << std::endl;
	_client.takeoffAsync()->waitOnLastTask();
}

void DroneRandomFlight::updateState(void)
{
	msr::airlib::MultirotorState state = _client.getMultirotorState();
	std::cout << "Position: "	//Eigen::Matrix<float, 3, 1>
		<< state.kinematics_estimated.pose.position.x() << ", "
		<< state.kinematics_estimated.pose.position.y() << ", "
		<< state.kinematics_estimated.pose.position.z() << std::endl;
	std::cout << "Orientation: "	//Eigen::Quaternionf
		<< state.kinematics_estimated.pose.orientation.w() << ", "
		<< state.kinematics_estimated.pose.orientation.x() << ", "
		<< state.kinematics_estimated.pose.orientation.y() << ", "
		<< state.kinematics_estimated.pose.orientation.z() << std::endl;
	std::cout << "state.collision.has_collided = " << (bool)state.collision.has_collided << std::endl;
}

int main(void) 
{
	DroneRandomFlight drone_random_flight;
	// drone_random_flight.startSampling();

	return 0;
}
