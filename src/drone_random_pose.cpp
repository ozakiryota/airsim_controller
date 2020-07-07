#include <iostream>
#include "api/RpcLibClientBase.hpp"
#include <fstream>
#include <opencv2/opencv.hpp>

class DroneRandomPose{
	private:
		/*client*/
		msr::airlib::RpcLibClientBase _client;
		/*state*/
		msr::airlib::Pose _pose;
		msr::airlib::ImuBase::Output _imu;;
		/*list*/
		std::vector _list_camera;
		/*csv*/
		std::ofstream _csvfile;
		/*parameter*/
		bool _save_data = true;
		int _num_sampling = 10;
		std::string _save_root_path = "/home/airsim_ws/airsim_controller/save";
		std::string _save_csv_path = _save_root_path + "/imu_camera.csv";

	public:
		DroneRandomPose();
		void clientInitialization(void);
		void csvInitialization(void);
		void startSampling(void);
		void randomPose(void);
		void saveData(void);
		void updateState(void);
		void eularToQuat(float r, float p, float y, Eigen::Quaternionf& q);
};

DroneRandomPose::DroneRandomPose()
{
	std::cout << "----- drone_random_pose -----" << std::endl;
	/*client*/
	clientInitialization();
	/*camera list*/
	_list_camera = {
		"front_center_custom"
	};
	/*csv*/
	if(_save_data)	csvInitialization();
}

void DroneRandomPose::clientInitialization(void)
{
	_client.confirmConnection();
	std::cout << "Reset" << std::endl;
	_client.reset();
	updateState();
}

void DroneRandomPose::csvInitialization(void)
{
	/*check*/
	std::ifstream ifs(_save_csv_path);
	if(ifs.is_open()){
		std::cout << _save_csv_path << " already exists" << std::endl;
		exit(1);
	}
	/*open*/
	_csvfile.open(_save_csv_path, std::ios::out);
	if(!_csvfile){
		std::cout << "Cannot open " << _save_csv_path << std::endl;
		exit(1);
	}
}

void DroneRandomPose::startSampling(void)
{
	std::cout << "Start sampling" << std::endl;

	for(int i=0; i<_num_sampling; ++i){
		std::cout << "--- sample " << i << " ---" << std::endl;
		randomPose();
		if(_save_data)	saveData();
		updateState();
	}
	_csvfile.close();
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
	msr::airlib::Pose pose = msr::airlib::Pose(position, orientation);
	std::cout << "Move to: " << std::endl
		<< " XYZ " << pose.position.x() << ", " << pose.position.y() << ", " << pose.position.z() << std::endl
		<< " RPY: " << roll << ", " << pitch << ", " << yaw << std::endl
		<< " Quat: " << pose.orientation.w() << ", " << pose.orientation.x() << ", " << pose.orientation.y() << ", " << pose.orientation.z() << std::endl;
	/*teleport*/
	_client.simSetVehiclePose(pose, true);
	// std::this_thread::sleep_for(std::chrono::seconds(1));
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void DroneRandomPose::saveData(void)
{
	/*file name*/
	std::vector<std::string> list_img_name(_list_camera.size());
	/*image request*/
	std::vector<msr::airlib::ImageCaptureBase::ImageRequest> list_request = {
		msr::airlib::ImageCaptureBase::ImageRequest("front_center_custom", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false)
	};
	std::vector<msr::airlib::ImageCaptureBase::ImageResponse> list_response = _client.simGetImages(list_request);
	/*access each image*/
	for(const msr::airlib::ImageCaptureBase::ImageResponse& response : list_response){
		std::string save_path = _save_root_path + "/" + std::to_string(response.time_stamp) + ".jpg";
		/*std::vector -> cv::mat*/
		cv::Mat img_cv = cv::Mat(response.height, response.width, CV_8UC3);
		for(int row=0; row<response.height; ++row){
			for(int col=0; col<response.width; ++col){
				img_cv.at<cv::Vec3b>(row, col)[0] = response.image_data_uint8[3*row*response.width + 3*col + 0];
				img_cv.at<cv::Vec3b>(row, col)[1] = response.image_data_uint8[3*row*response.width + 3*col + 1];
				img_cv.at<cv::Vec3b>(row, col)[2] = response.image_data_uint8[3*row*response.width + 3*col + 2];
			}
		}
		std::cout << "Save: " << save_path << std::endl;
		cv::imwrite(save_path, img_cv);              
		// std::cout << "size: " << response.image_data_uint8.size() << std::endl;
		// std::cout << "height: " << response.height << std::endl;
		// std::cout << "width: " << response.width << std::endl;
	}

	/*imu with other*/
	_csvfile 
		<< _imu.linear_acceleration.x << "," 
		<< _imu.linear_acceleration.y << "," 
		<< _imu.linear_acceleration.z << ","
		<< std::endl;
}

void DroneRandomPose::updateState(void)
{
	/*pose*/
	_pose = _client.simGetVehiclePose();
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
	/*imu*/
	_imu = _client.getImuData();
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
