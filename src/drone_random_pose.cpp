#include <iostream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

class DroneRandomPose{
	private:
		msr::airlib::MultirotorRpcLibClient _client;

	public:
		DroneRandomPose();
		void initialization(void);
		void startSampling(void);
		void randomPose(void);
		void printState(void);
		float computeL2Norm(float x, float y, float z);
};

DroneRandomPose::DroneRandomPose()
{
	/*initialize*/
	initialization();
}

void DroneRandomPose::initialization(void)
{
	std::cout << "Reset" << std::endl;
	_client.reset();
	std::cout << "Enable API control" << std::endl;
	_client.enableApiControl(true);
	/* std::cout << "Arm the drone" << std::endl; */
	/* _client.armDisarm(true); */
	/* printState(); */
	/* std::cout << "Take off" << std::endl; */
	/* _client.takeoffAsync()->waitOnLastTask(); */
	/* printState(); */
}

void DroneRandomPose::startSampling(void)
{
	std::cout << "Start sampling" << std::endl;

	const int num_sample = 10;
	for(int i=0; i<num_sample; ++i){
		std::cout << "--- sample " << i << " ---" << std::endl;
		printState();
		randomPose();
		_client.simPause(true);
		printState();
		_client.simPause(false);
	}
	std::cout << "Land" << std::endl;
	_client.landAsync()->waitOnLastTask();
}

void DroneRandomPose::randomPose(void)
{
	/*random*/
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<> urd_xy(-50.0, 50.0);
	std::uniform_real_distribution<> urd_z(-3.0, -2.0);
	/*set pose*/
	float x = urd_xy(mt);
	float y = urd_xy(mt);
	float z = urd_z(mt);
	Pose pose = Pose(Vector3r(x, y, z), Quaternionr(1, 0, 0, 0));
	/*teleport*/
	RpcLibClientBase client;
	client.simSetVehiclePose(pose, true);
}

void DroneRandomPose::printState(void)
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
	DroneRandomPose drone_random_pose;
	drone_random_pose.startSampling();

	return 0;
}
