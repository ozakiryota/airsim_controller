#include <iostream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

class DroneRandomPose{
	private:
		/*client*/
		RpcLibClientBase _client;
		/*parameter*/
		bool _save_image = false;
		std::string _save_root_path = "/home/airsim_ws/airsim_controller/save";

	public:
		DroneRandomPose();
		void initialization(void);
		void startSampling(void);
		void randomPose(void);
		void saveData(void);
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
		if(_save_image)	saveData();
		printPose();
	}
}

void DroneRandomPose::randomPose(void)
{
	/*parameter*/
	const float xy_range = 200.0;
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
		<< " XYZ " << pose.position.x() << ", " << pose.position.y() << ", " << pose.position.z() << std::endl
		<< " RPY: " << roll << ", " << pitch << ", " << yaw << std::endl
		<< " Quat: " << pose.orientation.w() << ", " << pose.orientation.x() << ", " << pose.orientation.y() << ", " << pose.orientation.z() << std::endl;
	/*teleport*/
	_client.simSetVehiclePose(pose, false);
	// std::this_thread::sleep_for(std::chrono::seconds(1));
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void DroneRandomPose::saveData(void)
{
	std::vector<msr::airlib::ImageCaptureBase::ImageRequest> list_request = {
		msr::airlib::ImageCaptureBase::ImageRequest("front_center_custom", msr::airlib::ImageCaptureBase::ImageType::Scene)
	};
	std::vector<msr::airlib::ImageCaptureBase::ImageResponse> list_response = _client.simGetImages(list_request);
	for(const msr::airlib::ImageCaptureBase::ImageResponse& response : list_response){
		std::string save_path = _save_root_path + "/" + std::to_string(response.time_stamp) + ".png";
		std::ofstream file(save_path, std::ios::binary);
		file.write(reinterpret_cast<const char*>(response.image_data_uint8.data()), response.image_data_uint8.size());
		file.close();
		std::cout << "Save: " << save_path << std::endl;
	}

}

void DroneRandomPose::printPose(void)
{
	/*pose*/
	msr::airlib::Pose pose = _client.simGetVehiclePose();
	std::cout << "Pose: " << std::endl;
	std::cout << " Position: "	//Eigen::Vector3f
		<< pose.position.x() << ", "
		<< pose.position.y() << ", "
		<< pose.position.z() << std::endl;
	std::cout << " Orientation: "	//Eigen::Quaternionf
		<< pose.orientation.w() << ", "
		<< pose.orientation.x() << ", "
		<< pose.orientation.y() << ", "
		<< pose.orientation.z() << std::endl;
	/*IMU*/
	msr::airlib::ImuBase::Output imu = _client.getImuData();
	std::cout << "IMU: " << std::endl;
	std::cout << " linear_acceleration: "	//Eigen::Vector3f
		<< imu.linear_acceleration.x() << ", "
		<< imu.linear_acceleration.y() << ", "
		<< imu.linear_acceleration.z() << std::endl;
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
