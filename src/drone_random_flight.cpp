#include <iostream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

int main() 
{
	// using namespace std;
	msr::airlib::MultirotorRpcLibClient client;

	std::cout << "Press Enter to enable API control" << std::endl;
	std::cin.get();
	client.enableApiControl(true);

	std::cout << "Press Enter to arm the drone" << std::endl;
	std::cin.get();
	client.armDisarm(true);

	std::cout << "Press Enter to takeoff" << std::endl;
	std::cin.get();
	client.takeoffAsync(5)->waitOnLastTask();

	std::cout << "Press Enter to rotation" << std::endl;
	std::cin.get();
	client.moveByRollPitchYawZAsync(M_PI/4.0, 0.0, 0.0, 0.0, 5.0)->waitOnLastTask();

	std::cout << "Press Enter to move 5 meters in x direction with 1 m/s velocity" << std::endl;
	std::cin.get();  
	auto position = client.getMultirotorState().getPosition(); // from current location
	client.moveToPositionAsync(position.x() + 5, position.y(), position.z(), 1)->waitOnLastTask();

	std::cout << "Press Enter to land" << std::endl;
	std::cin.get();
	client.landAsync()->waitOnLastTask();

	return 0;
}
