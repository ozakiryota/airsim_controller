#include <iostream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

class DroneRandomPose{
	private:
		RpcLibClientBase _client;

	public:
		DroneRandomPose();
		void initialization(void);
		void startSampling(void);
		void randomPose(void);
		void printPose(void);
		float computeL2Norm(float x, float y, float z);
};

DroneRandomPose::DroneRandomPose()
{
	/*initialize*/
	initialization();
}

void DroneRandomPose::initialization(void)
{
	_client.confirmConnection();
	_client.simSetCameraOrientation("camera0", Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0));
}

void DroneRandomPose::startSampling(void)
{
	std::cout << "Start sampling" << std::endl;

	const int num_sample = 10;
	for(int i=0; i<num_sample; ++i){
		std::cout << "--- sample " << i << " ---" << std::endl;
		printPose();
		randomPose();
		_client.simPause(true);
		printPose();
		std::this_thread::sleep_for(std::chrono::seconds(1));
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
	msr::airlib::Pose pose = Pose(Vector3r(x, y, z), Quaternionr(1, 0, 0, 0));
	/*teleport*/
	_client.simSetVehiclePose(pose, false);
}

void DroneRandomPose::printPose(void)
{
	msr::airlib::Pose pose = _client.simGetVehiclePose();
	std::cout << "Position: "	//Eigen::Matrix<float, 3, 1>
		<< pose.position.x() << ", "
		<< pose.position.y() << ", "
		<< pose.position.z() << std::endl;
	std::cout << "Orientation: "	//Eigen::Quaternionf
		<< pose.orientation.w() << ", "
		<< pose.orientation.x() << ", "
		<< pose.orientation.y() << ", "
		<< pose.orientation.z() << std::endl;
}

int main(void) 
{
	DroneRandomPose drone_random_pose;
	drone_random_pose.startSampling();

	return 0;
}
