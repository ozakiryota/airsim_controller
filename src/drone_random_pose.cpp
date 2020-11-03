#include <iostream>
#include "api/RpcLibClientBase.hpp"
#include <fstream>
#include <opencv2/opencv.hpp>
#include"cnpy.h"

class DroneRandomPose{
	private:
		/*client*/
		msr::airlib::RpcLibClientBase _client;
		/*state*/
		msr::airlib::Pose _pose;
		msr::airlib::ImuBase::Output _imu;;
		/*list*/
		std::vector<std::string> _list_camera;
		std::vector<msr::airlib::WorldSimApiBase::WeatherParameter> _list_weather;
		/*csv*/
		std::ofstream _csvfile;
		/*parameter-save*/
		const bool _save_data = true;
		const int _num_sampling = 100;
		const std::string _save_root_path = "/home/airsim_ws/airsim_controller/save/tmp";
		const std::string _save_csv_path = _save_root_path + "/imu_lidar_camera.csv";
		/*parameter-condition*/
		const bool _lidar_is_available = false;
		const bool _randomize_whether = true;
		/*parameter-pose*/
		const float _x_range = 200.0;	//Neighborhood: 200, SoccerField: 350
		const float _y_range = 200.0;	//Neighborhood: 200, SoccerField: 300
		const float _z_min = -3.0;
		const float _z_max = -2.0;
		const float _rp_range = M_PI/6.0;
		/*parameter-lidar*/
		int _num_rings = 32;
		int _points_per_ring = 1812;
		double _fov_upper_deg = 15;
		double _fov_lower_deg = -25;
		/*txt*/
		std::ofstream _txtfile;

	public:
		DroneRandomPose();
		void leaveParamNote(void);
		void clientInitialization(void);
		void csvInitialization(void);
		void startSampling(void);
		void randomWeather(void);
		void randomPose(void);
		void updateState(void);
		bool saveData(void);
		bool saveImages(std::vector<std::string>& list_img_name);
		bool saveLidarData(std::string& depthimg_name);
		void eularToQuat(float r, float p, float y, Eigen::Quaternionf& q);
};

DroneRandomPose::DroneRandomPose()
{
	std::cout << "----- drone_random_pose -----" << std::endl;
	/*parameter*/
	leaveParamNote();
	/*client*/
	clientInitialization();
	/*camera list*/
	_list_camera = {
		"camera_0"
	};
	/*
	_list_camera = {
		"camera_0",
		"camera_288",
		"camera_216",
		"camera_144",
		"camera_72"
	};
	*/
	/*csv*/
	if(_save_data)	csvInitialization();
}

void DroneRandomPose::leaveParamNote(void)
{
	/*open*/
	const std::string _save_txt_path = _save_root_path + "/param_note.txt";
	// _txtfile.open(_save_txt_path, std::ios::out);
	_txtfile.open(_save_txt_path, std::ios::app);
	if(!_txtfile){
		std::cout << "Cannot open " << _save_txt_path << std::endl;
		exit(1);
	}
	/*write*/
	_txtfile
		<< "----------" << std::endl
		<< "_randomize_whether" << ": " << (bool)_randomize_whether << std::endl
		<< "_num_sampling" << ": " << _num_sampling << std::endl
		<< "_x_range" << ": " << _x_range << std::endl
		<< "_y_range" << ": " << _y_range << std::endl
		<< "_z_min" << ": " << _z_min << std::endl
		<< "_z_max" << ": " << _z_max << std::endl
		<< "_rp_range" << ": " << _rp_range/M_PI*180.0 << std::endl;
	/*close*/
	_txtfile.close();
}

void DroneRandomPose::clientInitialization(void)
{
	/*connect*/
	_client.confirmConnection();
	/*reset*/
	std::cout << "Reset" << std::endl;
	_client.reset();
	/*pose*/
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	updateState();
	/*weather*/
	if(_randomize_whether)	_client.simEnableWeather(true);
	_list_weather = {
		msr::airlib::WorldSimApiBase::WeatherParameter::Rain,
		msr::airlib::WorldSimApiBase::WeatherParameter::Roadwetness,
		msr::airlib::WorldSimApiBase::WeatherParameter::Snow,
		msr::airlib::WorldSimApiBase::WeatherParameter::RoadSnow,
		msr::airlib::WorldSimApiBase::WeatherParameter::MapleLeaf,
		msr::airlib::WorldSimApiBase::WeatherParameter::RoadLeaf,
		msr::airlib::WorldSimApiBase::WeatherParameter::Dust,
		msr::airlib::WorldSimApiBase::WeatherParameter::Fog
		// msr::airlib::WorldSimApiBase::WeatherParameter::Enabled
	};
	/*time*/
	// _client.simSetTimeOfDay(true, "2018-02-12 15:20:00", false, 1000.0, 0.1, true);
}

void DroneRandomPose::csvInitialization(void)
{
	/*check*/
	// std::ifstream ifs(_save_csv_path);
	// if(ifs.is_open()){
	//	std::cout << _save_csv_path << " already exists" << std::endl;
	//	exit(1);
	// }
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

	for(int i=0; i<_num_sampling;){
		std::cout << "--- sample " << i << " ---" << std::endl;
		if(_randomize_whether)	randomWeather();
		randomPose();
		_client.simPause(true);
		updateState();
		if(_save_data){
			if(saveData())	++i;
		}
		else	++i;
		_client.simPause(false);
	}
	_csvfile.close();
}

void DroneRandomPose::randomWeather(void)
{
	/*reset*/
	for(size_t i=0; i<_list_weather.size(); ++i)	_client.simSetWeatherParameter(_list_weather[i], 0.0);
	/*random*/
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_int_distribution<> uid_index(0, _list_weather.size()-1);
	std::uniform_real_distribution<> urd_val(0.0, 1.0);
	for(int i=1; i<=uid_index(mt); ++i){
		int weather_index = uid_index(mt);
		double weather_val = urd_val(mt);
		_client.simSetWeatherParameter(_list_weather[weather_index], weather_val);
	}
}

void DroneRandomPose::randomPose(void)
{
	/*random*/
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<> urd_x(-_x_range, _x_range);
	std::uniform_real_distribution<> urd_y(-_y_range, _y_range);
	std::uniform_real_distribution<> urd_z(_z_min, _z_max);
	std::uniform_real_distribution<> urd_roll_pitch(-_rp_range, _rp_range);
	std::uniform_real_distribution<> urd_yaw(-M_PI, M_PI);
	/*set pose*/
	Eigen::Vector3f position(urd_x(mt), urd_y(mt), urd_z(mt));
	float roll = urd_roll_pitch(mt);
	float pitch = urd_roll_pitch(mt);
	float yaw = urd_yaw(mt);
	Eigen::Quaternionf orientation;
	eularToQuat(roll, pitch, yaw, orientation);
	msr::airlib::Pose goal = msr::airlib::Pose(position, orientation);
	std::cout << "Move to: " << std::endl
		<< " XYZ[m]: " 
			<< goal.position.x() << ", "
			<< goal.position.y() << ", "
			<< goal.position.z() << std::endl
		<< " RPY[deg]: "
			<< roll/M_PI*180.0 << ", "
			<< pitch/M_PI*180.0 << ", "
			<< yaw/M_PI*180.0 << std::endl
		<< " Quat: "
			<< goal.orientation.w() << ", "
			<< goal.orientation.x() << ", "
			<< goal.orientation.y() << ", "
			<< goal.orientation.z() << std::endl;
	/*teleport*/
	_client.simSetVehiclePose(goal, true);
	std::this_thread::sleep_for(std::chrono::milliseconds(200));
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

bool DroneRandomPose::saveData(void)
{
	/*image*/
	std::vector<std::string> list_img_name(_list_camera.size());
	if(!saveImages(list_img_name))	return false;

	/*lidar*/
	std::string depthimg_name;
	if(_lidar_is_available){
		if(!saveLidarData(depthimg_name))	return false;
	}

	/*imu (NEU) with other*/
	_csvfile 
		<< _imu.linear_acceleration.x() << "," 
		<< -_imu.linear_acceleration.y() << "," 
		<< -_imu.linear_acceleration.z();
	if(_lidar_is_available)	_csvfile << "," << depthimg_name;
	for(size_t i=0; i<list_img_name.size(); ++i){
		_csvfile << "," << list_img_name[i];
	}
	_csvfile << std::endl;

	return true;
}

bool DroneRandomPose::saveImages(std::vector<std::string>& list_img_name)
{
	/*request-responce*/
	std::vector<msr::airlib::ImageCaptureBase::ImageRequest> list_request(_list_camera.size());
	for(size_t i=0; i<_list_camera.size(); ++i){
		list_request[i] = msr::airlib::ImageCaptureBase::ImageRequest(_list_camera[i], msr::airlib::ImageCaptureBase::ImageType::Scene, false, false);
	}
	std::vector<msr::airlib::ImageCaptureBase::ImageResponse> list_response = _client.simGetImages(list_request);
	/*access each image*/
	for(size_t i=0; i<list_response.size(); ++i){
		list_img_name[i] = std::to_string(list_response[i].time_stamp) + "_" +  _list_camera[i] + ".jpg";
		std::string save_path = _save_root_path + "/" + list_img_name[i];
		/*check*/
		std::ifstream ifs(save_path);
		if(ifs.is_open()){
			std::cout << save_path << " already exists" << std::endl;
			return false;
		}
		/*std::vector -> cv::mat*/
		cv::Mat img_cv = cv::Mat(list_response[i].height, list_response[i].width, CV_8UC3);
		for(int row=0; row<list_response[i].height; ++row){
			for(int col=0; col<list_response[i].width; ++col){
				img_cv.at<cv::Vec3b>(row, col)[0] = list_response[i].image_data_uint8[3*row*list_response[i].width + 3*col + 0];
				img_cv.at<cv::Vec3b>(row, col)[1] = list_response[i].image_data_uint8[3*row*list_response[i].width + 3*col + 1];
				img_cv.at<cv::Vec3b>(row, col)[2] = list_response[i].image_data_uint8[3*row*list_response[i].width + 3*col + 2];
			}
		}
		std::cout << "Saved: " << save_path << std::endl;
		cv::imwrite(save_path, img_cv);              
		// std::cout << "size: " << list_response[i].image_data_uint8.size() << std::endl;
		// std::cout << "height: " << list_response[i].height << std::endl;
		// std::cout << "width: " << list_response[i].width << std::endl;
	}
	return true;
}

bool DroneRandomPose::saveLidarData(std::string& depthimg_name)
{
	/*resolution*/
	double angle_h_resolution = (_fov_upper_deg - _fov_lower_deg)/180.0*M_PI/(double)(_num_rings - 1);
	double angle_w_resolution = 2*M_PI/(double)_points_per_ring;
	/*initialize*/
	std::vector<double> mat(_num_rings*_points_per_ring, 0.0);
	std::cout << "mat.size() = " << mat.size() << std::endl;
	/*get*/
	msr::airlib::LidarData lidar_data = _client.getLidarData("");
	std::cout << "test: get" << std::endl;
	/*input*/
	for(size_t i=0; i<lidar_data.point_cloud.size(); i+=3){
		/*NED -> NEU*/
		double p_x = lidar_data.point_cloud[i];
		double p_y = -lidar_data.point_cloud[i+1];
		double p_z = -lidar_data.point_cloud[i+2];
		/*row*/
		double angle_h = atan2(p_z, sqrt(p_x*p_x + p_y*p_y));
		int row = (_fov_upper_deg/180.0*M_PI - angle_h)/angle_h_resolution;
		//if(row < 0 || row >= _num_rings){
		//	std::cout << "ERROR: row = " << row << std::endl;
		//	exit(1);
		//}
		/*col*/
		double angle_w = atan2(p_y, p_x);
		int col = (_points_per_ring - 1) - (int)((angle_w + M_PI)/angle_w_resolution);
		//if(col < 0 || col >= _points_per_ring){
		//	std::cout << "ERROR col" << std::endl;
		//	exit(1);
		//}
		/*depth*/
		double depth = sqrt(p_x*p_x + p_y*p_y);
		/*input*/
		mat[row*_num_rings + col] = depth;
	}
	std::cout << "test: input" << std::endl;
	/*path*/
	depthimg_name = std::to_string(lidar_data.time_stamp) + ".npy";
	std::string save_path = _save_root_path + "/" + depthimg_name;
	std::cout << "test: path: " << save_path << std::endl;
	/*check*/
	std::ifstream ifs(save_path);
	if(ifs.is_open()){
		std::cout << save_path << " already exists" << std::endl;
		return false;
	}
	std::cout << "test: check" << std::endl;
	/*save*/
	cnpy::npy_save(save_path, &mat[0], {(long unsigned int)_num_rings, (long unsigned int)_points_per_ring}, "w");
	std::cout << "Saved: " << save_path << std::endl;

	return true;
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
