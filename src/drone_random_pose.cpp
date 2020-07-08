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
		std::vector<std::string> _list_camera;
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
		"camera_0"
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
	//std::ifstream ifs(_save_csv_path);
	//if(ifs.is_open()){
	//	std::cout << _save_csv_path << " already exists" << std::endl;
	//	exit(1);
	//}
	/*open*/
	_csvfile.open(_save_csv_path, std::ios::app);
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
		_client.simPause(true);
		updateState();
		if(_save_data)	saveData();
		_client.simPause(false);
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
	msr::airlib::Pose goal = msr::airlib::Pose(position, orientation);
	std::cout << "Move to: " << std::endl
		<< " XYZ " << goal.position.x() << ", " << goal.position.y() << ", " << goal.position.z() << std::endl
		<< " RPY: " << roll << ", " << pitch << ", " << yaw << std::endl
		<< " Quat: " << goal.orientation.w() << ", " << goal.orientation.x() << ", " << goal.orientation.y() << ", " << goal.orientation.z() << std::endl;
	/*teleport*/
	_client.simSetVehiclePose(goal, true);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void DroneRandomPose::saveData(void)
{
	/*list*/
	std::vector<std::string> list_img_name(_list_camera.size());
	std::vector<msr::airlib::ImageCaptureBase::ImageRequest> list_request(_list_camera.size());
	/*image request-responce*/
	for(size_t i=0; i<_list_camera.size(); ++i){
		list_request[i] = msr::airlib::ImageCaptureBase::ImageRequest(_list_camera[i], msr::airlib::ImageCaptureBase::ImageType::Scene, false, false);
	}
	std::vector<msr::airlib::ImageCaptureBase::ImageResponse> list_response = _client.simGetImages(list_request);
	/*access each image*/
	for(size_t i=0; i<list_response.size(); ++i){
		list_img_name[i] = std::to_string(list_response[i].time_stamp) + "_" +  _list_camera[i] + ".jpg";
		std::string save_path = _save_root_path + "/" + list_img_name[i];
		/*std::vector -> cv::mat*/
		cv::Mat img_cv = cv::Mat(list_response[i].height, list_response[i].width, CV_8UC3);
		for(int row=0; row<list_response[i].height; ++row){
			for(int col=0; col<list_response[i].width; ++col){
				img_cv.at<cv::Vec3b>(row, col)[0] = list_response[i].image_data_uint8[3*row*list_response[i].width + 3*col + 0];
				img_cv.at<cv::Vec3b>(row, col)[1] = list_response[i].image_data_uint8[3*row*list_response[i].width + 3*col + 1];
				img_cv.at<cv::Vec3b>(row, col)[2] = list_response[i].image_data_uint8[3*row*list_response[i].width + 3*col + 2];
			}
		}
		std::cout << "Save: " << save_path << std::endl;
		cv::imwrite(save_path, img_cv);              
		// std::cout << "size: " << list_response[i].image_data_uint8.size() << std::endl;
		// std::cout << "height: " << list_response[i].height << std::endl;
		// std::cout << "width: " << list_response[i].width << std::endl;
	}

	/*imu with other*/
	_csvfile 
		<< _imu.linear_acceleration.x() << "," 
		<< _imu.linear_acceleration.y() << "," 
		<< -_imu.linear_acceleration.z() << ",";
	for(size_t i=0; i<list_img_name.size(); ++i){
		_csvfile << list_img_name[i];
	}
	_csvfile << std::endl;
}

void DroneRandomPose::updateState(void)
{
	/*pose*/
	_pose = _client.simGetVehiclePose();
	std::cout << "Pose: " << std::endl;
	std::cout << " Position: "	//Eigen::Vector3f
		<< _pose.position.x() << ", "
		<< _pose.position.y() << ", "
		<< _pose.position.z() << std::endl;
	std::cout << " Orientation: "	//Eigen::Quaternionf
		<< _pose.orientation.w() << ", "
		<< _pose.orientation.x() << ", "
		<< _pose.orientation.y() << ", "
		<< _pose.orientation.z() << std::endl;
	/*imu*/
	_imu = _client.getImuData();
	std::cout << "IMU: " << std::endl;
	std::cout << " linear_acceleration: "	//Eigen::Vector3f
		<< _imu.linear_acceleration.x() << ", "
		<< _imu.linear_acceleration.y() << ", "
		<< _imu.linear_acceleration.z() << std::endl;
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
