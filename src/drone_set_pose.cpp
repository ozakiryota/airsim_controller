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
		/*parameter*/
		const std::string _save_root_path = "/home/airsim_ws/airsim_controller/save/set_pose";
		/*txt*/
		std::ofstream _txtfile;

	public:
		DroneRandomPose();
		void clientInitialization(void);
		void interaction(void);
		void setPose(void);
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
	/*txt*/
	const std::string save_txt_path = _save_root_path + "/param_note.txt";
	_txtfile.open(save_txt_path, std::ios::out);
	if(!_txtfile){
		std::cout << "Cannot open " << save_txt_path << std::endl;
		exit(1);
	}
}

void DroneRandomPose::clientInitialization(void)
{
	/*connect*/
	_client.confirmConnection();
	/*reset*/
	std::cout << "Reset" << std::endl;
	_client.reset();
	/*pose*/
	msr::airlib::Pose goal = msr::airlib::Pose(Eigen::Vector3f(0.0, 0.0, 0.0), Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	updateState();
}

void DroneRandomPose::interaction(void)
{
	while(true){
		/*set pose*/
		setPose();
		_client.simPause(true);
		updateState();
		/*save*/
		std::string key_input;
		std::cout << "Do you want to save the pictures and data? (y or n)" << std::endl;
		std::cin >> key_input;
		if(key_input == "y")	saveData();
		else if(key_input == "n")	std::cout << "Dumped" << std::endl;
		/*repeat*/
		std::cout << "Do you want to set another pose? (y or n)" << std::endl;
		if(key_input == "y")	break;
	}
	/*close*/
	_txtfile.close();
}

void DroneRandomPose::setPose(void)
{
	/*set pose*/
	Eigen::Vector3f position;
	float roll;
	float pitch;
	float yaw;
	/*Interaction*/
	std::cout << "Enter: x = ?" << std::endl;
	std::cin >> position(0);
	std::cout << "Enter: y = ?" << std::endl;
	std::cin >> position(1);
	std::cout << "Enter: z = ?" << std::endl;
	std::cin >> position(2);
	std::cout << "Enter: roll = ?" << std::endl;
	std::cin >> roll;
	std::cout << "Enter: pitch = ?" << std::endl;
	std::cin >> pitch;
	std::cout << "Enter: yaw = ?" << std::endl;
	std::cin >> yaw;
	/*conversion*/
	Eigen::Quaternionf orientation;
	eularToQuat(roll, pitch, yaw, orientation);
	/*input*/
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

void DroneRandomPose::saveData(void)
{

	std::vector<std::string> list_img_name(_list_camera.size());
	std::vector<msr::airlib::ImageCaptureBase::ImageRequest> list_request(_list_camera.size());

	for(size_t i=0; i<_list_camera.size(); ++i){
		list_request[i] = msr::airlib::ImageCaptureBase::ImageRequest(_list_camera[i], msr::airlib::ImageCaptureBase::ImageType::Scene, false, false);
	}
	std::vector<msr::airlib::ImageCaptureBase::ImageResponse> list_response = _client.simGetImages(list_request);

	for(size_t i=0; i<list_response.size(); ++i){
		list_img_name[i] = _list_camera[i] + ".jpg";
		std::string save_path = _save_root_path + "/" + list_img_name[i];

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
	}

	_txtfile 
		<< _imu.linear_acceleration.x() << "," 
		<< -_imu.linear_acceleration.y() << "," 
		<< -_imu.linear_acceleration.z() << ",";
	_txtfile << std::endl;

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
	drone_random_pose.interaction();

	return 0;
}
