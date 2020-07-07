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
		void eularToQuat(float r, float p, float y, Eigen::Quaternionf& q);
};

DroneRandomPose::DroneRandomPose()
{
	/*initialize*/
	initialization();
}

void DroneRandomPose::initialization(void)
{
	_client.confirmConnection();
	std::cout << "Reset" << std::endl;
	_client.reset();
}

void DroneRandomPose::startSampling(void)
{
	std::cout << "Start sampling" << std::endl;

	const int num_sample = 10;
	for(int i=0; i<num_sample; ++i){
		std::cout << "--- sample " << i << " ---" << std::endl;
		randomPose();
		printPose();
	}
}

void DroneRandomPose::randomPose(void)
{
	/*parameter*/
	const float xy_range = 100.0;
	const float z_min = -3.0;
	const float z_max = -2.0;
	const float rp_range = M_PI/4.0;
	/*random*/
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<> urd_xy(-xy_range, xy_range);
	std::uniform_real_distribution<> urd_z(z_min, z_max);
	std::uniform_real_distribution<> urd_rp(-rp_range, rp_range);
	std::uniform_real_distribution<> urd_y(-M_PI, M_PI);
	/*set pose*/
	Eigen::Vector3f position(urd_xy(mt), urd_xy(mt), urd_z(mt));
	float roll = urd_rp(mt);
	float pitch = urd_rp(mt);
	float yaw = urd_y(mt);
	Eigen::Quaternionf orientation;
	eularToQuat(roll, pitch, yaw, orientation);
	msr::airlib::Pose pose = Pose(position, orientation);
	std::cout << "Move to: " << std::endl
		<< "XYZ " << pose.position.x() << ", " << pose.position.y() << ", " << pose.position.z() << std::endl
		<< "RPY: " << roll << ", " << pitch << ", " << yaw << std::endl
		<< "Quat: " << pose.orientation.w() << ", " << pose.orientation.x() << ", " << pose.orientation.y() << ", " << pose.orientation.z() << std::endl;
	/*teleport*/
	_client.simSetVehiclePose(pose, true);
	std::this_thread::sleep_for(std::chrono::seconds(1));
}

void DroneRandomPose::printPose(void)
{
	msr::airlib::Pose pose = _client.simGetVehiclePose();
	std::cout << "Position: "	//Eigen::Vector3f
		<< pose.position.x() << ", "
		<< pose.position.y() << ", "
		<< pose.position.z() << std::endl;
	std::cout << "Orientation: "	//Eigen::Quaternionf
		<< pose.orientation.w() << ", "
		<< pose.orientation.x() << ", "
		<< pose.orientation.y() << ", "
		<< pose.orientation.z() << std::endl;
}

void DroneRandomPose::eularToQuat(float r, float p, float y, Eigen::Quaternionf& q)
{
	q = Eigen::AngleAxisf(y, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(p, Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(r, Eigen::Vector3f::UnitX());
}

int main(void) 
{
	DroneRandomPose drone_random_pose;
	drone_random_pose.startSampling();

	return 0;
}
