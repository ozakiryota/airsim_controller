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
		/*world*/
		std::vector<msr::airlib::WorldSimApiBase::WeatherParameter> weather_list_;
		/*sensor*/
		msr::airlib::Pose pose_;
		msr::airlib::ImuBase::Output imu_;
		std::vector<msr::airlib::ImageCaptureBase::ImageRequest> camera_request_list_;
		/*csv*/
		std::ofstream ofs_csv_;
		/*parameter-save*/
		bool save_data_ = true;
		bool overwrite_ = true;
		int num_sampling_ = 100;
		std::string save_dir_ = "../save/tmp";
		std::string save_csv_path_ = save_dir_ + "/file_list.csv";
		/*parameter-condition*/
		bool randomize_whether_ = true;
		int wait_time_msec_ = 200;
		/*parameter-pose*/
		float min_x_ = -200.0;	//Neighborhood: -200, SoccerField: -350
		float max_x_ = 200.0;	//Neighborhood: 200, SoccerField: 350
		float min_y_ = -200.0;	//Neighborhood: -200, SoccerField: -300
		float max_y_ = 200.0;	//Neighborhood: 200, SoccerField: 300
		float min_z_ = -3.0;
		float max_z_ = -2.0;
		float rp_range_deg_ = 30.0;
		float min_yaw_deg_ = -180.0;
		float max_yaw_deg_ = 180.0;
		/*parameter-lidar*/
		bool lidar_is_available_ = false;
		int num_rings_ = 32;
		int points_per_ring_ = 1812;
		float fov_upper_deg_ = 15;
		float fov_lower_deg_ = -25;
		/*function*/
		bool getParameters();
		void initializeClient();
		void leaveParamNote();
		void initializeCsv();
		void randomizeWeather();
		void randomizePose();
		void eularToQuat(float r, float p, float y, Eigen::Quaternionf& q);
		void updatePose();
		bool saveData();
		bool getCameraData(std::vector<std::string>& save_filename_list, std::vector<cv::Mat>& save_data_list);
		bool getLidarData(std::string& save_filename, std::vector<float>& save_data);
		float degToRad(float deg);

	public:
		DroneRandomPose();
		void sample();
};

DroneRandomPose::DroneRandomPose()
{
	std::cout << "----- drone_random_pose -----" << std::endl;
	/*parameter*/
	getParameters();
	/*client*/
	initializeClient();
	/*file*/
	if(save_data_){
		if(overwrite_)	std::filesystem::remove_all(save_dir_);
		std::filesystem::create_directory(save_dir_);
		leaveParamNote();
		initializeCsv();
	}
}

bool DroneRandomPose::getParameters()
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
	if(param_json.contains("overwrite"))	overwrite_ = param_json["overwrite"];
	std::cout << "overwrite_ = " << (bool)overwrite_ << std::endl;
	if(param_json.contains("num_sampling"))	num_sampling_ = param_json["num_sampling"];
	std::cout << "num_sampling_ = " << num_sampling_ << std::endl;
	if(param_json.contains("randomize_whether"))	randomize_whether_ = param_json["randomize_whether"];
	std::cout << "randomize_whether_ = " << randomize_whether_ << std::endl;
	if(param_json.contains("wait_time_msec"))	wait_time_msec_ = param_json["wait_time_msec"];
	std::cout << "wait_time_msec_ = " << wait_time_msec_ << std::endl;
	if(param_json.contains("min_x"))	min_x_ = param_json["min_x"];
	std::cout << "min_x_ = " << min_x_ << std::endl;
	if(param_json.contains("max_x"))	max_x_ = param_json["max_x"];
	std::cout << "max_x_ = " << max_x_ << std::endl;
	if(param_json.contains("min_y"))	min_y_ = param_json["min_y"];
	std::cout << "min_y_ = " << min_y_ << std::endl;
	if(param_json.contains("max_y"))	max_y_ = param_json["max_y"];
	std::cout << "max_y_ = " << max_y_ << std::endl;
	if(param_json.contains("min_z"))	min_z_ = param_json["min_z"];
	std::cout << "min_z_ = " << min_z_ << std::endl;
	if(param_json.contains("max_z"))	max_z_ = param_json["max_z"];
	std::cout << "max_z_ = " << max_z_ << std::endl;
	if(param_json.contains("rp_range_deg"))	rp_range_deg_ = param_json["rp_range_deg"];
	std::cout << "rp_range_deg_ = " << rp_range_deg_ << std::endl;
	if(param_json.contains("min_yaw_deg"))	min_yaw_deg_ = param_json["min_yaw_deg"];
	std::cout << "min_yaw_deg_ = " << min_yaw_deg_ << std::endl;
	if(param_json.contains("max_yaw_deg"))	max_yaw_deg_ = param_json["max_yaw_deg"];
	std::cout << "max_yaw_deg_ = " << max_yaw_deg_ << std::endl;
	if(param_json.contains("lidar_is_available"))	lidar_is_available_ = param_json["lidar_is_available"];
	std::cout << "lidar_is_available_ = " << lidar_is_available_ << std::endl;
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
	for(size_t i = 0; ; i++){
		std::string param_name = "scene_camera_" + std::to_string(i);
		if(param_json.contains(param_name)){
			std::string tmp_camera_name = param_json[param_name];
			camera_request_list_.push_back(msr::airlib::ImageCaptureBase::ImageRequest(tmp_camera_name, msr::airlib::ImageCaptureBase::ImageType::Scene, false, false));
		}
		else	break;
	}
	for(size_t i = 0; ; i++){
		std::string param_name = "sgmnt_camera_" + std::to_string(i);
		if(param_json.contains(param_name)){
			std::string tmp_camera_name = param_json[param_name];
			camera_request_list_.push_back(msr::airlib::ImageCaptureBase::ImageRequest(tmp_camera_name, msr::airlib::ImageCaptureBase::ImageType::Segmentation, false, false));
		}
		else	break;
	}

	return true;
}

void DroneRandomPose::initializeClient()
{
	/*connect*/
	client_.confirmConnection();
	/*reset*/
	std::cout << "Reset" << std::endl;
	client_.reset();
	/*pose*/
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	updatePose();
	/*weather*/
	if(randomize_whether_)	client_.simEnableWeather(true);
	weather_list_ = {
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

void DroneRandomPose::leaveParamNote()
{
	/*open*/
	std::ofstream txtfile;
	const std::string _save_txt_path = save_dir_ + "/param_note.txt";
	txtfile.open(_save_txt_path, std::ios::app);
	if(!txtfile){
		std::cout << "Cannot open " << _save_txt_path << std::endl;
		exit(true);
	}
	/*write*/
	txtfile << "----------" << std::endl
		<< "randomize_whether_" << ": " << (bool)randomize_whether_ << std::endl
		<< "num_sampling_" << ": " << num_sampling_ << std::endl
		<< "min_x_" << ": " << min_x_ << std::endl
		<< "max_x_" << ": " << max_x_ << std::endl
		<< "min_y_" << ": " << min_y_ << std::endl
		<< "max_y_" << ": " << max_y_ << std::endl
		<< "min_z_" << ": " << min_z_ << std::endl
		<< "max_z_" << ": " << max_z_ << std::endl
		<< "rp_range_deg_" << ": " << rp_range_deg_ << std::endl
		<< "min_yaw_deg_" << ": " << min_yaw_deg_ << std::endl
		<< "max_yaw_deg_" << ": " << max_yaw_deg_ << std::endl;
	/*close*/
	txtfile.close();
}

void DroneRandomPose::initializeCsv()
{
	ofs_csv_.open(save_csv_path_, std::ios::app);
	if(!ofs_csv_){
		std::cout << "Cannot open " << save_csv_path_ << std::endl;
		exit(true);
	}
}

void DroneRandomPose::sample()
{
	std::cout << "Start sampling" << std::endl;

	for(int i=0; i<num_sampling_;){
		std::cout << "--- sample " << i + 1 << " / " << num_sampling_ << " ---" << std::endl;
		if(randomize_whether_)	randomizeWeather();
		randomizePose();
		client_.simPause(true);
		updatePose();
		if(save_data_){
			if(saveData())	++i;
		}
		else	++i;
		client_.simPause(false);
	}
	ofs_csv_.close();
}

void DroneRandomPose::randomizeWeather()
{
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_int_distribution<> uid_index(0, 1);
	std::uniform_real_distribution<> urd_val(0.0, 1.0);
	for(const msr::airlib::WorldSimApiBase::WeatherParameter& weather : weather_list_){
		if(uid_index(mt)){
			float weather_val = urd_val(mt);
			client_.simSetWeatherParameter(weather, weather_val);
		}
		else	client_.simSetWeatherParameter(weather, 0);
	}
}

void DroneRandomPose::randomizePose()
{
	/*random*/
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<> urd_x(min_x_, max_x_);
	std::uniform_real_distribution<> urd_y(min_y_, max_y_);
	std::uniform_real_distribution<> urd_z(min_z_, max_z_);
	std::uniform_real_distribution<> urd_roll_pitch(-degToRad(rp_range_deg_), degToRad(rp_range_deg_));
	std::uniform_real_distribution<> urd_yaw(degToRad(min_yaw_deg_), degToRad(max_yaw_deg_));
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
			<< roll / M_PI * 180.0 << ", "
			<< pitch / M_PI * 180.0 << ", "
			<< yaw / M_PI * 180.0 << std::endl
		<< " Quat: "
			<< goal.orientation.w() << ", "
			<< goal.orientation.x() << ", "
			<< goal.orientation.y() << ", "
			<< goal.orientation.z() << std::endl;
	/*teleport*/
	client_.simSetVehiclePose(goal, true);
	// std::this_thread::sleep_for(std::chrono::milliseconds(wait_time_msec_));
	client_.simContinueForTime(wait_time_msec_ * 1e-3);
}

void DroneRandomPose::eularToQuat(float r, float p, float y, Eigen::Quaternionf& q)
{
	q = Eigen::AngleAxisf(y, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(p, Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(r, Eigen::Vector3f::UnitX());
}

void DroneRandomPose::updatePose()
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
}

bool DroneRandomPose::saveData()
{
	/*get-imu*/
	imu_ = client_.getImuData();
	std::string imu_filename = std::to_string(imu_.time_stamp) + "_imu.json";
	/*get-camera*/
	std::vector<std::string> camera_filename_list;
	std::vector<cv::Mat> camera_data_list;
	if(!camera_request_list_.empty()){
		if(!getCameraData(camera_filename_list, camera_data_list))	return false;
	}
	/*get lidar*/
	std::string lidar_filename;
	std::vector<float> lidar_data(num_rings_ * points_per_ring_, -1);
	if(lidar_is_available_){
		if(!getLidarData(lidar_filename, lidar_data))	return false;
	}

	/*save-imu*/
    nlohmann::json imu_data;
	imu_data["angular_velocity"]["x"] = imu_.angular_velocity.x();
	imu_data["angular_velocity"]["y"] = -imu_.angular_velocity.y();
	imu_data["angular_velocity"]["z"] = -imu_.angular_velocity.z();
	imu_data["linear_acceleration"]["x"] = imu_.linear_acceleration.x();
	imu_data["linear_acceleration"]["y"] = -imu_.linear_acceleration.y();
	imu_data["linear_acceleration"]["z"] = -imu_.linear_acceleration.z();
	std::string save_imu_path = save_dir_ + "/" + imu_filename;
	std::ofstream imu_json(save_imu_path);
	imu_json << imu_data;
    imu_json.close();
	ofs_csv_ << imu_filename << ",";
	std::cout << "Saved: " << save_imu_path << std::endl;
	/*save-camera*/
	for(size_t i = 0; i < camera_filename_list.size(); i++){
		std::string save_camera_path = save_dir_ + "/" + camera_filename_list[i];
		cv::imwrite(save_camera_path, camera_data_list[i]);
		ofs_csv_ << camera_filename_list[i]  << ",";
		std::cout << "Saved: " << save_camera_path << std::endl;
	}
	/*save-lidar*/
	if(lidar_is_available_){
		std::string save_lidar_path = save_dir_ + "/" + lidar_filename;
		cnpy::npy_save(save_lidar_path, &lidar_data[0], {(long unsigned int)num_rings_, (long unsigned int)points_per_ring_}, "w");
		ofs_csv_ << lidar_filename;
		std::cout << "Saved: " << save_lidar_path << std::endl;
	}
	ofs_csv_ << std::endl;

	return true;
}

bool DroneRandomPose::getCameraData(std::vector<std::string>& save_filename_list, std::vector<cv::Mat>& save_data_list)
{
	/*get*/
	std::vector<msr::airlib::ImageCaptureBase::ImageResponse> response_list = client_.simGetImages(camera_request_list_);
	/*convert*/
	for(const msr::airlib::ImageCaptureBase::ImageResponse& response : response_list){
		/*check-file*/
		std::string save_filename = std::to_string(response.time_stamp) + "_" +  response.camera_name + ".jpg";
		std::string save_path = save_dir_ + "/" + save_filename;
		std::ifstream ifs(save_path);
		if(ifs.is_open()){
			std::cout << save_path << " already exists" << std::endl;
			return false;
		}
		/*check-timestamp*/
		msr::airlib::TTimePoint time_diff_sec = (response.time_stamp - imu_.time_stamp) * 1e-9;
		if(time_diff_sec > wait_time_msec_ * 1e-3){
			std::cout << "time_diff_sec = " << time_diff_sec << " > " << wait_time_msec_ * 1e-3 << std::endl;
			return false;
		}
		/*std::vector -> cv::mat*/
		cv::Mat img_cv = cv::Mat(response.height, response.width, CV_8UC3);
		for(int row = 0; row < response.height; ++row){
			for(int col = 0; col < response.width; ++col){
				img_cv.at<cv::Vec3b>(row, col)[0] = response.image_data_uint8[3 * row * response.width + 3 * col + 0];
				img_cv.at<cv::Vec3b>(row, col)[1] = response.image_data_uint8[3 * row * response.width + 3 * col + 1];
				img_cv.at<cv::Vec3b>(row, col)[2] = response.image_data_uint8[3 * row * response.width + 3 * col + 2];
			}
		}
		/*append*/
		save_filename_list.push_back(save_filename);
		save_data_list.push_back(img_cv);
	}
	return true;
}

bool DroneRandomPose::getLidarData(std::string& save_filename, std::vector<float>& save_data)
{
	/*get*/
	msr::airlib::LidarData lidar_data = client_.getLidarData("");
	/*check-file*/
	save_filename = std::to_string(lidar_data.time_stamp) + ".npy";
	std::string save_path = save_dir_ + "/" + save_filename;
	std::ifstream ifs(save_path);
	if(ifs.is_open()){
		std::cout << save_path << " already exists" << std::endl;
		return false;
	}
	/*check-timestamp*/
	msr::airlib::TTimePoint time_diff_sec = (lidar_data.time_stamp - imu_.time_stamp) * 1e-9;
	if(time_diff_sec > wait_time_msec_ * 1e-3){
		std::cout << "time_diff_sec = " << time_diff_sec << " > " << wait_time_msec_ * 1e-3 << std::endl;
		return false;
	}
	/*resolution*/
	float angle_h_resolution = (fov_upper_deg_ - fov_lower_deg_) / 180.0 * M_PI / (float)(num_rings_ - 1);
	float angle_w_resolution = 2 * M_PI / (float)points_per_ring_;
	/*convert*/
	for(size_t i = 0; i < lidar_data.point_cloud.size(); i += 3){
		/*NED -> NEU*/
		float p_x = lidar_data.point_cloud[i];
		float p_y = -lidar_data.point_cloud[i+1];
		float p_z = -lidar_data.point_cloud[i+2];
		/*row*/
		float angle_h = atan2(p_z, sqrt(p_x * p_x + p_y * p_y));
		int row = (fov_upper_deg_ / 180.0 * M_PI - angle_h) / angle_h_resolution;
		if(row < 0 || row >= num_rings_){
			std::cout << "ERROR: row = " << row << std::endl;
			// exit(true);
			return false;
		}
		/*col*/
		float angle_w = atan2(p_y, p_x);
		int col = (points_per_ring_ - 1) - (int)((angle_w + M_PI) / angle_w_resolution);
		if(col < 0 || col >= points_per_ring_){
			std::cout << "ERROR: col" << std::endl;
			// exit(true);
			return false;
		}
		/*depth*/
		float depth = sqrt(p_x * p_x + p_y * p_y);
		/*input*/
		save_data[row * points_per_ring_ + col] = depth;
	}
	return true;
}

float DroneRandomPose::degToRad(float deg)
{
	float rad = deg / 180.0 * M_PI;
	rad = atan2(sin(rad), cos(rad));
	return rad;
}

int main() 
{
	DroneRandomPose drone_random_pose;
	drone_random_pose.sample();

	return 0;
}
