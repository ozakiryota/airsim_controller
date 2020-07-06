#include <iostream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

void printPosition(const msr::airlib::MultirotorRpcLibClient& client)
{
	Eigen::Matrix<float, 3, 1> position = client.getMultirotorState().getPosition();
	std::cout << "Position: "
		<< position.x() << ", "
		<< position.y() << ", "
		<< position.z() << std::endl;
}

int main() 
{
	msr::airlib::MultirotorRpcLibClient client;
	client.reset();

	std::cout << "Enable API control" << std::endl;
	client.enableApiControl(true);

	std::cout << "Arm the drone" << std::endl;
	client.armDisarm(true);

	std::cout << "Take off" << std::endl;
	client.takeoffAsync(5.0)->waitOnLastTask();

	Eigen::Matrix<float, 3, 1> position = client.getMultirotorState().getPosition();

	std::cout << "Rotation" << std::endl;
	client.moveByRollPitchYawZAsync(M_PI/6.0, 0.0, 0.0, position.z(), 1.0)->waitOnLastTask();

	client.simPause(true);
	printPosition(client);
	std::cout << "Press enter to start sampling" << std::endl;
	std::cin.get();
	client.simPause(false);
	const int num_samle = 10;
	for(int i=0; i<num_samle; ++i){
		std::cout << "samle " << i << std::endl;
		position = client.getMultirotorState().getPosition();
		msr::airlib::CollisionInfo collision_info = client.simGetCollisionInfo();
		std::cout << "collision_info.has_collided = " << (bool)collision_info.has_collided << std::endl;

		double x = position.x() + 1.0;
		double y = position.y() + 1.0;
		double z = position.z() - 1.0;
		double vel = 1.0;
		client.moveToPositionAsync(x, y, z, vel)->waitOnLastTask();
		std::cout << "Move to ("
			<< x << ", "
			<< y << ", "
			<< z << ") with "
			<< vel << "[m/s] "
			<< std::endl;
	}

	std::cout << "Land" << std::endl;
	client.landAsync()->waitOnLastTask();

	return 0;
}
