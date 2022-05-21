#include <iostream>
#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>
#include <opencv4/opencv2/opencv.hpp>

#include "api/RpcLibClientBase.hpp"
#include "cnpy.h"

class DroneRandomPose{
	private:
		/*client*/
		msr::airlib::RpcLibClientBase client_;
		/*state*/
		msr::airlib::Pose pose_;
		msr::airlib::ImuBase::Output imu_;
		/*list*/
		std::vector<std::string> list_camera_;
		std::vector<msr::airlib::WorldSimApiBase::WeatherParameter> list_weather_;
		/*csv*/
		std::ofstream ofs_csv_;
		/*parameter-save*/
		bool save_data_ = true;
		int num_sampling_ = 100;
		std::string save_dir_ = "../save/tmp";
		std::string save_csv_path_ = save_dir_ + "/imu.csv";
		/*parameter-condition*/
		bool lidar_is_available_ = false;
		bool randomize_whether_ = true;
		int wait_time_msec_ = 200;
		/*parameter-pose*/
		float x_range_ = 200.0;	//Neighborhood: 200, SoccerField: 350
		float y_range_ = 200.0;	//Neighborhood: 200, SoccerField: 300
		float z_min_ = -3.0;
		float z_max_ = -2.0;
		float rp_range_deg_ = 30.0;
		/*parameter-lidar*/
		int num_rings_ = 32;
		int points_per_ring_ = 1812;
		double fov_upper_deg_ = 15;
		double fov_lower_deg_ = -25;

	public:
		DroneRandomPose();
		bool getParameters(void);
		void leaveParamNote(void);
		void clientInitialization(void);
		void csvInitialization(void);
		void startSampling(void);
		void randomWeather(void);
		void randomPose(void);
		void updateState(void);
		bool saveData(void);
		bool saveImages(std::vector<std::string>& list_save_colorimage_name, std::vector<std::string>& list_save_colorimage_path, std::vector<cv::Mat>& list_colorimage_cv);
		bool saveLidarData(std::string& save_depthimg_name, std::string& save_depthimg_path, std::vector<double>& depthimage_mat);
		double degToRad(double deg);
		void eularToQuat(float r, float p, float y, Eigen::Quaternionf& q);
};

DroneRandomPose::DroneRandomPose()
{
	std::cout << "----- drone_random_pose -----" << std::endl;
	/*parameter*/
	getParameters();
	/*client*/
	clientInitialization();
	/*camera list*/
	list_camera_ = {
		"camera_0"
	};
	/*
	list_camera_ = {
		"camera_0",
		"camera_288",
		"camera_216",
		"camera_144",
		"camera_72"
	};
	*/
	/*csv*/
	if(lidar_is_available_)	save_csv_path_.insert(save_csv_path_.size() - std::string(".csv").size(), "_lidar");
	if(list_camera_.size() > 0)	save_csv_path_.insert(save_csv_path_.size() - std::string(".csv").size(), "_camera");
	if(save_data_){
		leaveParamNote();
		csvInitialization();
	}
}

bool DroneRandomPose::getParameters(void)
{
	/*open*/
	std::string json_path = "../config/drone_random_pose.json";
	std::ifstream ifs(json_path, std::ios::in);
	if(!ifs){
		std::cout << "Fail to open " << json_path << ", thus default parameters are used." << std::endl;
		return false;
	}
	nlohmann::json param_json;
	ifs >> param_json;
	/*get*/
	if(param_json.contains("save_data"))	save_data_ = param_json["save_data"];
	std::cout << "save_data_ = " << (bool)save_data_ << std::endl;
	if(param_json.contains("num_sampling"))	num_sampling_ = param_json["num_sampling"];
	std::cout << "num_sampling_ = " << num_sampling_ << std::endl;
	if(param_json.contains("lidar_is_available"))	lidar_is_available_ = param_json["lidar_is_available"];
	std::cout << "lidar_is_available_ = " << lidar_is_available_ << std::endl;
	if(param_json.contains("randomize_whether"))	randomize_whether_ = param_json["randomize_whether"];
	std::cout << "randomize_whether_ = " << randomize_whether_ << std::endl;
	if(param_json.contains("wait_time_msec"))	wait_time_msec_ = param_json["wait_time_msec"];
	std::cout << "wait_time_msec_ = " << wait_time_msec_ << std::endl;
	if(param_json.contains("x_range"))	x_range_ = param_json["x_range"];
	std::cout << "x_range_ = " << x_range_ << std::endl;
	if(param_json.contains("y_range"))	y_range_ = param_json["y_range"];
	std::cout << "y_range_ = " << y_range_ << std::endl;
	if(param_json.contains("z_min"))	z_min_ = param_json["z_min"];
	std::cout << "z_min_ = " << z_min_ << std::endl;
	if(param_json.contains("z_max"))	z_max_ = param_json["z_max"];
	std::cout << "z_max_ = " << z_max_ << std::endl;
	if(param_json.contains("rp_range_deg"))	rp_range_deg_ = param_json["rp_range_deg"];
	std::cout << "rp_range_deg_ = " << rp_range_deg_ << std::endl;
	if(lidar_is_available_){
		if(param_json.contains("num_rings"))	num_rings_ = param_json["num_rings"];
		std::cout << "num_rings_ = " << num_rings_ << std::endl;
		if(param_json.contains("points_per_ring"))	points_per_ring_ = param_json["points_per_ring"];
		std::cout << "points_per_ring_ = " << points_per_ring_ << std::endl;
		if(param_json.contains("fov_upper_deg"))	fov_upper_deg_ = param_json["fov_upper_deg"];
		std::cout << "fov_upper_deg_ = " << fov_upper_deg_ << std::endl;
		if(param_json.contains("fov_lower_deg"))	fov_lower_deg_ = param_json["fov_lower_deg"];
		std::cout << "fov_lower_deg_ = " << fov_lower_deg_ << std::endl;
	}

	return true;
}

void DroneRandomPose::leaveParamNote(void)
{
	/*open*/
	std::ofstream txtfile;
	const std::string _save_txt_path = save_dir_ + "/param_note.txt";
	// txtfile.open(_save_txt_path, std::ios::out);
	txtfile.open(_save_txt_path, std::ios::app);
	if(!txtfile){
		std::cout << "Cannot open " << _save_txt_path << std::endl;
		exit(1);
	}
	/*write*/
	txtfile
		<< "----------" << std::endl
		<< "randomize_whether_" << ": " << (bool)randomize_whether_ << std::endl
		<< "num_sampling_" << ": " << num_sampling_ << std::endl
		<< "x_range_" << ": " << x_range_ << std::endl
		<< "y_range_" << ": " << y_range_ << std::endl
		<< "z_min_" << ": " << z_min_ << std::endl
		<< "z_max_" << ": " << z_max_ << std::endl
		<< "rp_range_deg_" << ": " << rp_range_deg_ << std::endl;
	/*close*/
	txtfile.close();
}

void DroneRandomPose::clientInitialization(void)
{
	/*connect*/
	client_.confirmConnection();
	/*reset*/
	std::cout << "Reset" << std::endl;
	client_.reset();
	/*pose*/
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	updateState();
	/*weather*/
	if(randomize_whether_)	client_.simEnableWeather(true);
	list_weather_ = {
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
	// client_.simSetTimeOfDay(true, "2020-01-01 17:00:00", false, 0.01, 1.0, true);
}

void DroneRandomPose::csvInitialization(void)
{
	/*check*/
	// std::ifstream ifs(save_csv_path_);
	// if(ifs.is_open()){
	//	std::cout << save_csv_path_ << " already exists" << std::endl;
	//	exit(1);
	// }
	/*mkdir*/
	std::filesystem::remove_all(save_dir_);
	std::filesystem::create_directory(save_dir_);
	/*open*/
	ofs_csv_.open(save_csv_path_, std::ios::app);
	if(!ofs_csv_){
		std::cout << "Cannot open " << save_csv_path_ << std::endl;
		exit(1);
	}
}

void DroneRandomPose::startSampling(void)
{
	std::cout << "Start sampling" << std::endl;

	for(int i=0; i<num_sampling_;){
		std::cout << "--- sample " << i << " / " << num_sampling_ << " ---" << std::endl;
		if(randomize_whether_)	randomWeather();
		randomPose();
		client_.simPause(true);
		updateState();
		if(save_data_){
			if(saveData())	++i;
		}
		else	++i;
		client_.simPause(false);
	}
	ofs_csv_.close();
}

void DroneRandomPose::randomWeather(void)
{
	/*reset*/
	for(size_t i=0; i<list_weather_.size(); ++i)	client_.simSetWeatherParameter(list_weather_[i], 0.0);
	/*random*/
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_int_distribution<> uid_index(0, list_weather_.size()-1);
	std::uniform_real_distribution<> urd_val(0.0, 1.0);
	for(int i=1; i<=uid_index(mt); ++i){
		int weather_index = uid_index(mt);
		double weather_val = urd_val(mt);
		client_.simSetWeatherParameter(list_weather_[weather_index], weather_val);
	}
}

void DroneRandomPose::randomPose(void)
{
	/*random*/
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<> urd_x(-x_range_, x_range_);
	std::uniform_real_distribution<> urd_y(-y_range_, y_range_);
	std::uniform_real_distribution<> urd_z(z_min_, z_max_);
	std::uniform_real_distribution<> urd_roll_pitch(-degToRad(rp_range_deg_), degToRad(rp_range_deg_));
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
	client_.simSetVehiclePose(goal, true);
	std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_msec_));
}

void DroneRandomPose::updateState(void)
{
	/*pose*/
	pose_ = client_.simGetVehiclePose();
	std::cout << "Pose: " << std::endl;
	std::cout << " Position: "	//Eigen::Vector3f
		<< pose_.position.x() << ", "
		<< pose_.position.y() << ", "
		<< pose_.position.z() << std::endl;
	std::cout << " Orientation: "	//Eigen::Quaternionf
		<< pose_.orientation.w() << ", "
		<< pose_.orientation.x() << ", "
		<< pose_.orientation.y() << ", "
		<< pose_.orientation.z() << std::endl;
	/*imu*/
	imu_ = client_.getImuData();
	std::cout << "IMU: " << std::endl;
	std::cout << " linear_acceleration: "	//Eigen::Vector3f
		<< imu_.linear_acceleration.x() << ", "
		<< imu_.linear_acceleration.y() << ", "
		<< imu_.linear_acceleration.z() << std::endl;
}

bool DroneRandomPose::saveData(void)
{
	/*get image*/
	std::vector<std::string> list_save_colorimage_name(list_camera_.size());
	std::vector<std::string> list_save_colorimage_path(list_camera_.size());
	std::vector<cv::Mat> list_colorimage_cv(list_camera_.size());
	if(!list_camera_.empty()){
		if(!saveImages(list_save_colorimage_name, list_save_colorimage_path, list_colorimage_cv))	return false;
	}
	/*get lidar*/
	std::string save_depthimg_name;
	std::string save_depthimg_path;
	std::vector<double> depthimage_mat(num_rings_*points_per_ring_, -1);
	if(lidar_is_available_){
		if(!saveLidarData(save_depthimg_name, save_depthimg_path, depthimage_mat))	return false;
	}
	/*save*/
	for(size_t i=0; i<list_save_colorimage_path.size(); ++i){
		std::cout << "Saved: " << list_save_colorimage_path[i] << std::endl;
		cv::imwrite(list_save_colorimage_path[i], list_colorimage_cv[i]);
	}
	if(lidar_is_available_){
		cnpy::npy_save(save_depthimg_path, &depthimage_mat[0], {(long unsigned int)num_rings_, (long unsigned int)points_per_ring_}, "w");
		std::cout << "Saved: " << save_depthimg_path << std::endl;
	}
	/*imu (NEU) with other information*/
	// std::cout << imu_.time_stamp << std::endl;
	ofs_csv_ 
		<< imu_.linear_acceleration.x() << "," 
		<< -imu_.linear_acceleration.y() << "," 
		<< -imu_.linear_acceleration.z();
	if(lidar_is_available_)	ofs_csv_ << "," << save_depthimg_name;
	for(size_t i=0; i<list_save_colorimage_name.size(); ++i){
		ofs_csv_ << "," << list_save_colorimage_name[i];
	}
	ofs_csv_ << std::endl;

	return true;
}

bool DroneRandomPose::saveImages(std::vector<std::string>& list_save_colorimage_name, std::vector<std::string>& list_save_colorimage_path, std::vector<cv::Mat>& list_colorimage_cv)
{
	/*request-responce*/
	std::vector<msr::airlib::ImageCaptureBase::ImageRequest> list_request(list_camera_.size());
	for(size_t i=0; i<list_camera_.size(); ++i){
		list_request[i] = msr::airlib::ImageCaptureBase::ImageRequest(list_camera_[i], msr::airlib::ImageCaptureBase::ImageType::Scene, false, false);
	}
	std::vector<msr::airlib::ImageCaptureBase::ImageResponse> list_response = client_.simGetImages(list_request);
	/*access each image*/
	for(size_t i=0; i<list_response.size(); ++i){
		list_save_colorimage_name[i] = std::to_string(list_response[i].time_stamp) + "_" +  list_camera_[i] + ".jpg";
		std::string save_path = save_dir_ + "/" + list_save_colorimage_name[i];
		/*check-file*/
		std::ifstream ifs(save_path);
		if(ifs.is_open()){
			std::cout << save_path << " already exists" << std::endl;
			return false;
		}
		/*check-timestamp*/
		if((list_response[i].time_stamp - imu_.time_stamp)*1e-9 > wait_time_msec_*1e-3){
			std::cout << "(list_response[i].time_stamp - imu_.time_stamp)*1e-9 = " << (list_response[i].time_stamp - imu_.time_stamp)*1e-9 << " > " << wait_time_msec_*1e-3 << std::endl;
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
		list_save_colorimage_path[i] = save_path;
		list_colorimage_cv[i] = img_cv;
		// std::cout << "size: " << list_response[i].image_data_uint8.size() << std::endl;
		// std::cout << "height: " << list_response[i].height << std::endl;
		// std::cout << "width: " << list_response[i].width << std::endl;
	}
	return true;
}

bool DroneRandomPose::saveLidarData(std::string& save_depthimg_name, std::string& save_depthimg_path, std::vector<double>& depthimage_mat)
{
	/*resolution*/
	double angle_h_resolution = (fov_upper_deg_ - fov_lower_deg_)/180.0*M_PI/(double)(num_rings_ - 1);
	double angle_w_resolution = 2*M_PI/(double)points_per_ring_;
	/*get*/
	msr::airlib::LidarData lidar_data = client_.getLidarData("");
	/*path*/
	save_depthimg_name = std::to_string(lidar_data.time_stamp) + ".npy";
	save_depthimg_path = save_dir_ + "/" + save_depthimg_name;
	/*check-file*/
	std::ifstream ifs(save_depthimg_path);
	if(ifs.is_open()){
		std::cout << save_depthimg_path << " already exists" << std::endl;
		return false;
	}
	/*check-timestamp*/
	if((lidar_data.time_stamp - imu_.time_stamp)*1e-9 > wait_time_msec_*1e-3){
		std::cout << "(lidar_data.time_stamp - imu_.time_stamp)*1e-9 = " << (lidar_data.time_stamp - imu_.time_stamp)*1e-9 << " > " << wait_time_msec_*1e-3 << std::endl;
		return false;
	}
	/*input*/
	for(size_t i=0; i<lidar_data.point_cloud.size(); i+=3){
		/*NED -> NEU*/
		double p_x = lidar_data.point_cloud[i];
		double p_y = -lidar_data.point_cloud[i+1];
		double p_z = -lidar_data.point_cloud[i+2];
		/*row*/
		double angle_h = atan2(p_z, sqrt(p_x*p_x + p_y*p_y));
		int row = (fov_upper_deg_/180.0*M_PI - angle_h)/angle_h_resolution;
		if(row < 0 || row >= num_rings_){
			std::cout << "ERROR: row = " << row << std::endl;
			// exit(1);
			return false;
		}
		/*col*/
		double angle_w = atan2(p_y, p_x);
		int col = (points_per_ring_ - 1) - (int)((angle_w + M_PI)/angle_w_resolution);
		if(col < 0 || col >= points_per_ring_){
			std::cout << "ERROR col" << std::endl;
			// exit(1);
			return false;
		}
		/*depth*/
		double depth = sqrt(p_x*p_x + p_y*p_y);
		/*input*/
		depthimage_mat[row*points_per_ring_ + col] = depth;
	}
	/*test*/
	//int test_counter = 0;
	//std::cout << "lidar_data.point_cloud.size()/3 = " << lidar_data.point_cloud.size()/3 << std::endl;
	//std::cout << "depthimage_mat.size() = " << depthimage_mat.size() << std::endl;
	//for(size_t i=0; i<depthimage_mat.size(); ++i){
	//	if(depthimage_mat[i] == 0)  ++test_counter;
	//}
	//std::cout << "test_counter = " << test_counter << std::endl;

	return true;
}

double DroneRandomPose::degToRad(double deg)
{
	double rad = deg / 180.0 * M_PI;
	rad = atan2(sin(rad), cos(rad));
	return rad;
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
