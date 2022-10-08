#include <iostream>
#include <nlohmann/json.hpp>

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

class DroneRandomFlight{
	private:
		/*client*/
		msr::airlib::MultirotorRpcLibClient client_;
		/*path*/
		std::vector<Eigen::Vector3f> waypoints_;
		/*parameter*/
		int num_sampling_ = 10;
		float min_x_ = -10.0;
		float max_x_ = 10.0;
		float min_y_ = -10.0;
		float max_y_ = 10.0;
		float min_z_ = -3.0;
		float max_z_ = -2.0;
		float velocity_ = 5.0;

	public:
		DroneRandomFlight();
		bool getParameters(void);
		void clientInitialization(void);
		void getRandomWeypoints(void);
		void flight(void);
};

DroneRandomFlight::DroneRandomFlight()
{
	std::cout << "----- drone_random_flight -----" << std::endl;
	/*parameter*/
	getParameters();
	/*client*/
	clientInitialization();
	/*waypoints*/
	getRandomWeypoints();
}

bool DroneRandomFlight::getParameters(void)
{
	/*open*/
	std::string json_path = "../config/drone_random_flight.json";
	std::ifstream ifs(json_path, std::ios::in);
	if(!ifs){
		std::cout << "Fail to open " << json_path << ", thus default parameters are used." << std::endl;
		return false;
	}
	nlohmann::json param_json;
	ifs >> param_json;
	/*get*/
	if(param_json.contains("num_sampling"))	num_sampling_ = param_json["num_sampling"];
	std::cout << "num_sampling_ = " << num_sampling_ << std::endl;
	if(param_json.contains("min_x"))	min_x_ = param_json["min_x"];
	std::cout << "min_x_ = " << min_x_ << std::endl;
	if(param_json.contains("max_x"))	min_x_ = param_json["max_x"];
	std::cout << "max_x_ = " << max_x_ << std::endl;
	if(param_json.contains("min_y"))	min_y_ = param_json["min_y"];
	std::cout << "min_y_ = " << min_y_ << std::endl;
	if(param_json.contains("max_y"))	min_y_ = param_json["max_y"];
	std::cout << "max_y_ = " << max_y_ << std::endl;
	if(param_json.contains("min_z"))	min_z_ = param_json["min_z"];
	std::cout << "min_z_ = " << min_z_ << std::endl;
	if(param_json.contains("max_z"))	min_z_ = param_json["max_z"];
	std::cout << "max_z_ = " << max_z_ << std::endl;
	if(param_json.contains("velocity"))	velocity_ = param_json["velocity"];
	std::cout << "velocity_ = " << velocity_ << std::endl;

	return true;
}

void DroneRandomFlight::clientInitialization(void)
{
	/*connect*/
	client_.confirmConnection();
	/*reset*/
	client_.reset();
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	/*set*/
	client_.enableApiControl(true);
	client_.armDisarm(true);
}

void DroneRandomFlight::getRandomWeypoints(void)
{
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<> urd_x(min_y_, max_x_);
	std::uniform_real_distribution<> urd_y(min_y_, max_y_);
	std::uniform_real_distribution<> urd_z(min_z_, max_z_);

	waypoints_.resize(num_sampling_);
	for(int i = 0; i < num_sampling_; i++)	waypoints_[i] = Eigen::Vector3f(urd_x(mt), urd_y(mt), urd_z(mt));
}

void DroneRandomFlight::flight(void)
{
	client_.takeoffAsync()->waitOnLastTask();

	msr::airlib::YawMode yaw_mode;
	yaw_mode.is_rate = false;
	yaw_mode.yaw_or_rate = 0;
	client_.moveOnPathAsync(
		waypoints_,
		velocity_,
		msr::airlib::Utils::max<float>(),
		msr::airlib::DrivetrainType::ForwardOnly,
		yaw_mode
	)->waitOnLastTask();
}

int main(void) 
{
	DroneRandomFlight drone_random_flight;
	drone_random_flight.flight();

	return 0;
}