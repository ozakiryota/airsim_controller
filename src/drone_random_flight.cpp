#include <iostream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

int main() 
{
	// using namespace std;
	msr::airlib::MultirotorRpcLibClient client;

	std::cout << "Enable API control" << std::endl;
	client.enableApiControl(true);

	std::cout << "Arm the drone" << std::endl;
	client.armDisarm(true);

	std::cout << "Take off" << std::endl;
	client.takeoffAsync(5.0)->waitOnLastTask();

	Eigen::Matrix<float, 3, 1> position = client.getMultirotorState().getPosition();

	std::cout << "Rotation" << std::endl;
	client.moveByRollPitchYawZAsync(M_PI/4.0, 0.0, 0.0, position.z(), 5.0)->waitOnLastTask();

	const int num_samle = 100;
	for(int i=0; i<num_samle; ++i)
		position = client.getMultirotorState().getPosition();
		double x = position.x() + 1.0;
		double y = position.y() + 1.0;
		double z = position.z() + 1.0;
		double vel = 5.0;
		client.moveToPositionAsync(x, y, y, vel)->waitOnLastTask();
		std::cout << "Move to ("
			<< x << ", "
			<< y << ", "
			<< z << ") with"
			<< vel << "[m/s] "
			<< std::endl;
	}

	std::cout << "Land" << std::endl;
	client.landAsync()->waitOnLastTask();

	return 0;
}
