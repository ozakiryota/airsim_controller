#include <iostream>
#include <fstream>

#include "api/RpcLibClientBase.hpp"

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include"cnpy.h"

class DroneSetPose{
	private:
		/*client*/
		msr::airlib::RpcLibClientBase _client;
		/*state*/
		msr::airlib::Pose _pose;
		msr::airlib::ImuBase::Output _imu;;
		/*list*/
		std::vector<std::string> _list_camera;
		/*path*/
		const std::string _save_root_path = "/home/airsim_ws/airsim_controller/save/set_pose";
		/*parameter-lidar*/
		const int _num_rings = 32;
		const int _points_per_ring = 1812;
		const double _fov_upper_deg = 15;
		const double _fov_lower_deg = -25;

	public:
		DroneSetPose();
		void clientInitialization(void);
		void interaction(void);
		void setPose(void);
		void updateState(void);
		void saveData(void);
		void saveImage(void);
		void savePC(void);
		void savePoseAndIMU(void);
		void eularToQuat(float r, float p, float y, Eigen::Quaternionf& q);
};

DroneSetPose::DroneSetPose()
{
	std::cout << "----- drone_random_pose -----" << std::endl;
	/*client*/
	clientInitialization();
	/*camera list*/
	_list_camera = {
		"camera_0"
	};
	/*
	_list_camera = {
		"camera_0",
		"camera_72",
		"camera_144",
		"camera_216",
		"camera_288"
	};
	*/
}

void DroneSetPose::clientInitialization(void)
{
	/*connect*/
	_client.confirmConnection();
	/*reset*/
	std::cout << "Reset" << std::endl;
	_client.reset();
	/*pose*/
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	updateState();
}

void DroneSetPose::interaction(void)
{
	bool is_done = false;
	while(!is_done){
		/*set pose*/
		setPose();
		// _client.simPause(true);
		updateState();
		/*save*/
		std::string save_y_or_n;
		while(save_y_or_n != "y" && save_y_or_n != "n"){
			std::cout << "Enter: Do you want to save the pictures and data? (y or n)" << std::endl;
			std::cin >> save_y_or_n;
			if(save_y_or_n == "y")	saveData();
			else if(save_y_or_n == "n")	std::cout << "Dumped" << std::endl;
			else	std::cout << "Type y or n" << std::endl;
		}
		/*repeat*/
		std::string continue_y_or_n;
		while(continue_y_or_n != "y" && continue_y_or_n != "n"){
			std::cout << "Enter: Do you want to continue setting another pose? (y or n)" << std::endl;
			std::cin >> continue_y_or_n;
			if(continue_y_or_n == "y")	is_done = false;
			else if(continue_y_or_n == "n")	is_done = true;
			else	std::cout << "Type y or n" << std::endl;
		}
		// _client.simPause(false);
	}
}

void DroneSetPose::setPose(void)
{
	/*set pose*/
	Eigen::Vector3f position;
	float roll;
	float pitch;
	float yaw;
	/*Interaction*/
	std::cout << "Enter: x [m] = ?" << std::endl;
	std::cin >> position(0);
	std::cout << "Enter: y [m] = ?" << std::endl;
	std::cin >> position(1);
	std::cout << "Enter: z [m] = ?" << std::endl;
	std::cin >> position(2);
	std::cout << "Enter: roll [deg] = ?" << std::endl;
	std::cin >> roll;
	std::cout << "Enter: pitch [deg] = ?" << std::endl;
	std::cin >> pitch;
	std::cout << "Enter: yaw [deg] = ?" << std::endl;
	std::cin >> yaw;
	/*conversion*/
	Eigen::Quaternionf orientation;
	eularToQuat(roll/180.0*M_PI, pitch/180.0*M_PI, yaw/180.0*M_PI, orientation);
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

void DroneSetPose::updateState(void)
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

void DroneSetPose::saveData(void)
{
	saveImage();
	savePC();
	savePoseAndIMU();
}

void DroneSetPose::saveImage(void)
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
		list_img_name[i] = _list_camera[i] + ".jpg";
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
	}
}

void DroneSetPose::savePC(void)
{
	/*get*/
	msr::airlib::LidarData lidar_data = _client.getLidarData("");
	if(lidar_data.point_cloud.size() == 0)	return;
	/*vector->pcl (NEU)*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
	for(size_t i=0; i<lidar_data.point_cloud.size(); i+=3){
		pcl::PointXYZ tmp;
		tmp.x = lidar_data.point_cloud[i];
		tmp.y = -lidar_data.point_cloud[i+1];
		tmp.z = -lidar_data.point_cloud[i+2];
		pc->points.push_back(tmp);
	}
	/*width x height*/
	pc->width = pc->points.size();
	pc->height = 1;
	/*save PCD*/
	const std::string save_pcd_path = _save_root_path + "/lidar.pcd";
	pcl::io::savePCDFileASCII(save_pcd_path, *pc);

	/*pcl->mat*/
	std::vector<double> mat(_num_rings*_points_per_ring, 0.0);
	double angle_h_resolution = (_fov_upper_deg - _fov_lower_deg)/180.0*M_PI/(double)(_num_rings - 1);
	double angle_w_resolution = 2*M_PI/(double)_points_per_ring;
	for(size_t i=0; i<pc->points.size(); ++i){
		/*row*/
		double angle_h = atan2(pc->points[i].z, sqrt(pc->points[i].x*pc->points[i].x + pc->points[i].y*pc->points[i].y));
		int row = (_fov_upper_deg/180.0*M_PI - angle_h)/angle_h_resolution;
		/*col*/
		double angle_w = atan2(pc->points[i].y, pc->points[i].x);
		int col = (_points_per_ring - 1) - (int)((angle_w + M_PI)/angle_w_resolution);
		/*depth*/
		double depth = sqrt(pc->points[i].x*pc->points[i].x + pc->points[i].y*pc->points[i].y);
		/*input*/
		mat[row*_points_per_ring + col] = depth;
	}
	/*save NPY*/
	const std::string save_npy_path = _save_root_path + "/lidar.npy";
	cnpy::npy_save(save_npy_path, &mat[0], {_num_rings, _points_per_ring}, "w");
}

void DroneSetPose::savePoseAndIMU(void)
{
	/*open*/
	std::ofstream txtfile;
	const std::string save_txt_path = _save_root_path + "/param_note.txt";
	txtfile.open(save_txt_path, std::ios::out);
	if(!txtfile){
		std::cout << "Cannot open " << save_txt_path << std::endl;
		exit(1);
	}
	/*write*/
	txtfile << "Pose: " << std::endl;
	txtfile << " Position: "
		<< _pose.position.x() << ", "
		<< _pose.position.y() << ", "
		<< _pose.position.z() << std::endl;
	txtfile << " Orientation: "
		<< _pose.orientation.w() << ", "
		<< _pose.orientation.x() << ", "
		<< _pose.orientation.y() << ", "
		<< _pose.orientation.z() << std::endl;
	txtfile << "IMU: " << std::endl;
	txtfile << " linear_acceleration: "
		<< _imu.linear_acceleration.x() << "," 
		<< -_imu.linear_acceleration.y() << "," 
		<< -_imu.linear_acceleration.z() << std::endl;
	/*close*/
	txtfile.close();
}

void DroneSetPose::eularToQuat(float r, float p, float y, Eigen::Quaternionf& q)
{
	q = Eigen::AngleAxisf(y, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(p, Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(r, Eigen::Vector3f::UnitX());
}

int main(void) 
{
	DroneSetPose drone_random_pose;
	drone_random_pose.interaction();

	return 0;
}
