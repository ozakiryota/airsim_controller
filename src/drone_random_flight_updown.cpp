#include <iostream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

class DroneRandomFlight{
	private:
		msr::airlib::MultirotorRpcLibClient _client;

	public:
		DroneRandomFlight();
		void initialization(void);
		void startSampling(void);
		void randomGlobalMove(void);
		void randomRotation(void);
		void hover(void);
		void printState(void);
		float computeL2Norm(float x, float y, float z);
};

DroneRandomFlight::DroneRandomFlight()
{
	/*initialize*/
	initialization();
}

void DroneRandomFlight::initialization(void)
{
	std::cout << "Reset" << std::endl;
	_client.reset();
	std::cout << "Enable API control" << std::endl;
	_client.enableApiControl(true);
	std::cout << "Arm the drone" << std::endl;
	_client.armDisarm(true);
	printState();
	std::cout << "Take off" << std::endl;
	_client.takeoffAsync()->waitOnLastTask();
	printState();
}

void DroneRandomFlight::startSampling(void)
{
	std::cout << "Start sampling" << std::endl;

	const int num_sample = 10;
	for(int i=0; i<num_sample; ++i){
		std::cout << "--- sample " << i << " ---" << std::endl;
		printState();
		randomGlobalMove();
		_client.simPause(true);
		_client.simPause(false);
	}
	std::cout << "Land" << std::endl;
	_client.landAsync()->waitOnLastTask();
}

void DroneRandomFlight::randomGlobalMove(void)
{
	/*get state*/
	msr::airlib::MultirotorState state = _client.getMultirotorState();
	/*set goal*/
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<> urd_xy(-50.0, 50.0);
	std::uniform_real_distribution<> urd_z(-3.0, -2.0);
	float x = urd_xy(mt);
	float y = urd_xy(mt);
	float z = urd_z(mt);
	/*set velocity*/
	float dist = computeL2Norm(
		x - state.kinematics_estimated.pose.position.x(),
		y - state.kinematics_estimated.pose.position.y(),
		0.0
	);
	float vel = dist/2.0;
	/*print*/
	std::cout << "Move to ("
		<< x << ", "
		<< y << ", "
		<< z << ") with "
		<< vel << "[m/s] "
		<< std::endl;
	/*up to sky*/
	const float sky_height = -20.0;
	 _client.moveToZAsync(sky_height, std::abs(sky_height)/2.0)->waitOnLastTask();
	/*move on xy plane*/
	_client.moveToPositionAsync(x, y, sky_height, vel)->waitOnLastTask();
	/*down to ground*/
	_client.moveToPositionAsync(x, y, z, std::abs(sky_height)/2.0)->waitOnLastTask();
	printState();
	hover();
}

void DroneRandomFlight::randomRotation(void)
{
	msr::airlib::MultirotorState state = _client.getMultirotorState();

	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<> urd_rpy(-M_PI, M_PI);

	float r = urd_rpy(mt);
	float p = urd_rpy(mt);
	float y = urd_rpy(mt);
	float z = state.kinematics_estimated.pose.position.z();
	float duration = 1.0;

	_client.moveByRollPitchYawZAsync(r, p, y, z, duration)->waitOnLastTask();
}

void DroneRandomFlight::hover(void)
{
	msr::airlib::MultirotorState state = _client.getMultirotorState();
	float x = state.kinematics_estimated.pose.position.x();
	float y = state.kinematics_estimated.pose.position.y();
	float z = state.kinematics_estimated.pose.position.z();
	float duration = 0.5;
	float vel = 2.0;

	_client.moveByRollPitchYawZAsync(0.0, 0.0, 0.0, z, duration)->waitOnLastTask();
	_client.moveToPositionAsync(x, y, z, vel)->waitOnLastTask();
	_client.moveByRollPitchYawZAsync(0.0, 0.0, 0.0, z, duration)->waitOnLastTask();
}

void DroneRandomFlight::printState(void)
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

float DroneRandomFlight::computeL2Norm(float x, float y, float z)
{
	return sqrt(x*x + y*y + z*z);
}

int main(void) 
{
	/* msr::airlib::MultirotorRpcLibClient client; */
	/* client.reset(); */
    /*  */
	/* std::cout << "Enable API control" << std::endl; */
	/* client.enableApiControl(true); */
    /*  */
	/* std::cout << "Arm the drone" << std::endl; */
	/* client.armDisarm(true); */
    /*  */
	/* std::cout << "Take off" << std::endl; */
	/* client.takeoffAsync(5.0)->waitOnLastTask(); */
    /*  */
	/* Eigen::Matrix<float, 3, 1> position = client.getMultirotorState().getPosition(); */
    /*  */
	/* std::cout << "Rotation" << std::endl; */
	/* client.moveByRollPitchYawZAsync(M_PI/6.0, 0.0, 0.0, position.z(), 1.0)->waitOnLastTask(); */
    /*  */
	/* client.simPause(true); */
	/* printPosition(client); */
	/* std::cout << "Press enter to start sampling" << std::endl; */
	/* std::cin.get(); */
	/* client.simPause(false); */
	/* const int num_samle = 10; */
	/* for(int i=0; i<num_samle; ++i){ */
	/* 	std::cout << "samle " << i << std::endl; */
	/* 	position = client.getMultirotorState().getPosition(); */
	/* 	msr::airlib::CollisionInfo collision_info = client.simGetCollisionInfo(); */
	/* 	std::cout << "collision_info.has_collided = " << (bool)collision_info.has_collided << std::endl; */
    /*  */
	/* 	double x = position.x() + 1.0; */
	/* 	double y = position.y() + 1.0; */
	/* 	double z = position.z() - 1.0; */
	/* 	double vel = 8.0; */
	/* 	client.moveToPositionAsync(x, y, z, vel)->waitOnLastTask(); */
	/* 	std::cout << "Move to (" */
	/* 		<< x << ", " */
	/* 		<< y << ", " */
	/* 		<< z << ") with " */
	/* 		<< vel << "[m/s] " */
	/* 		<< std::endl; */
	/* } */
    /*  */
	/* std::cout << "Land" << std::endl; */
	/* client.landAsync()->waitOnLastTask(); */

	DroneRandomFlight drone_random_flight;
	drone_random_flight.startSampling();

	return 0;
}
