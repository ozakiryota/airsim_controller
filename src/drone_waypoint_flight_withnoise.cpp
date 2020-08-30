#include <iostream>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

class DroneWayPointFlight{
	private:
		msr::airlib::MultirotorRpcLibClient _client;
		std::vector<Eigen::Vector3f> _waypoints;
		std::vector<Eigen::Vector3f> _path;
		double _height = -2.5;
		double _path_resolution = 2.5;
		double _noise_xy = 2.5;
		double _noise_z = 0.5;
		double _velocity = 15.0;
		double _cutting_corner = 5.0;

	public:
		DroneWayPointFlight();
		void setWayPoints(void);
		void devidePath(void);
		void addNoise(Eigen::Vector3f& point);
		void printPath(void);
		void clientInitialization(void);
		void startFlight(void);
};

DroneWayPointFlight::DroneWayPointFlight()
{
	std::cout << "----- drone_waypoint_flight_withnoise -----" << std::endl;
	/*initialize*/
	setWayPoints();
	devidePath();
	// addNoise();
	printPath();
	clientInitialization();
}

void DroneWayPointFlight::setWayPoints(void)
{
	const int course_number = 0;
	const int loop = 1;
	const double side_length = 127.5;
	const double middle_point = 79.0;
	/*Neighborhood course 0*/
	if(course_number == 0){
		_waypoints = {
			Eigen::Vector3f(0.0, 0.0, _height),
			Eigen::Vector3f(side_length, 0.0, _height)
		};
		for(int i=0; i<loop; ++i){
			_waypoints.push_back(Eigen::Vector3f(side_length, side_length, _height));
			_waypoints.push_back(Eigen::Vector3f(-side_length, side_length, _height));
			_waypoints.push_back(Eigen::Vector3f(-side_length, -side_length, _height));
			_waypoints.push_back(Eigen::Vector3f(side_length, -side_length, _height));
		}
		_waypoints.push_back(Eigen::Vector3f(side_length, 0.0, _height));
		_waypoints.push_back(Eigen::Vector3f(0.0, 0.0, _height));
	}
	/*Neighborhood course 1*/
	else if(course_number == 1){
		_waypoints = {
			Eigen::Vector3f(0.0, 0.0, _height)
		};
		for(int i=0; i<loop; ++i){
			_waypoints.push_back(Eigen::Vector3f(side_length, 0.0, _height));
			_waypoints.push_back(Eigen::Vector3f(side_length, side_length, _height));
			_waypoints.push_back(Eigen::Vector3f(-side_length, side_length, _height));
			_waypoints.push_back(Eigen::Vector3f(-side_length, -side_length, _height));
			_waypoints.push_back(Eigen::Vector3f(side_length, -side_length, _height));
			_waypoints.push_back(Eigen::Vector3f(side_length, 0.0, _height));
			_waypoints.push_back(Eigen::Vector3f(middle_point, 0.0, _height));
			_waypoints.push_back(Eigen::Vector3f(middle_point, -side_length, _height));
			_waypoints.push_back(Eigen::Vector3f(0.0, -side_length, _height));
			_waypoints.push_back(Eigen::Vector3f(0.0, side_length, _height));
			_waypoints.push_back(Eigen::Vector3f(-side_length, side_length, _height));
			_waypoints.push_back(Eigen::Vector3f(-side_length, 0.0, _height));
			_waypoints.push_back(Eigen::Vector3f(0.0, 0.0, _height));
		}
	}
	/*SoccerField course 2*/
	else if(course_number == 2){
		_waypoints = {
			Eigen::Vector3f(0.0, 0.0, _height)
		};
		for(int i=0; i<loop; ++i){
			_waypoints.push_back(Eigen::Vector3f(0, 0, _height));
			_waypoints.push_back(Eigen::Vector3f(-158, 130, _height));
			_waypoints.push_back(Eigen::Vector3f(-158, -43, _height));
			_waypoints.push_back(Eigen::Vector3f(-14, -44, _height));
			_waypoints.push_back(Eigen::Vector3f(10, -42, _height));
			_waypoints.push_back(Eigen::Vector3f(72, -65, _height));
			_waypoints.push_back(Eigen::Vector3f(79, -69, _height));
			_waypoints.push_back(Eigen::Vector3f(127, -28, _height));
			_waypoints.push_back(Eigen::Vector3f(225, -26, _height));
			_waypoints.push_back(Eigen::Vector3f(230, 7, _height));
			_waypoints.push_back(Eigen::Vector3f(160, 16, _height));
			_waypoints.push_back(Eigen::Vector3f(127, 61, _height));
			_waypoints.push_back(Eigen::Vector3f(-32, 63, _height));
			_waypoints.push_back(Eigen::Vector3f(-64, 91, _height));
			_waypoints.push_back(Eigen::Vector3f(-62, 157, _height));
		}
	}
}

void DroneWayPointFlight::devidePath(void)
{
	for(size_t i=0; i<_waypoints.size()-1; ++i){
		Eigen::Vector3f delta = _waypoints[i+1] - _waypoints[i];
		int points_per_line = int(delta.norm()/_path_resolution);
		if(points_per_line > 0){
			Eigen::Vector3f step = delta/(double)points_per_line;
			for(size_t j=1; j<points_per_line; ++j){
				Eigen::Vector3f point = _waypoints[i] + j*step;
				addNoise(point);
				double dist_to_next = (_waypoints[i+1] - point).norm();
				if(dist_to_next > _cutting_corner)	_path.push_back(point);
			}
		}
		_path.push_back(_waypoints[i+1]);
		_path[_path.size()-1] -= (_waypoints[i+1] - _waypoints[i]).normalized()*_cutting_corner;
	}
}

void DroneWayPointFlight::addNoise(Eigen::Vector3f& point)
{
	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<> urd_xy(-_noise_xy, _noise_xy);
	std::uniform_real_distribution<> urd_z(-_noise_z, _noise_z);

	point(0) += urd_xy(mt);
	point(1) += urd_xy(mt);
	point(2) += urd_z(mt);
}

void DroneWayPointFlight::printPath(void)
{
	for(size_t i=0; i<_path.size(); ++i){
		std::cout << i << ": ("
			<< _path[i](0) << ", "
			<< _path[i](1) << ", "
			<< _path[i](2) << ")"
		<< std::endl;
	}
	std::cout << "_path.size() = " << _path.size() << std::endl;
}

void DroneWayPointFlight::clientInitialization(void)
{
	/*connect*/
	_client.confirmConnection();
	/*reset*/
	std::cout << "Reset" << std::endl;
	_client.reset();
	/*control*/
	std::cout << "Enable API control" << std::endl;
	_client.enableApiControl(true);
	std::cout << "Arm the drone" << std::endl;
	_client.armDisarm(true);
	std::cout << "Take off" << std::endl;
	_client.takeoffAsync()->waitOnLastTask();
}

void DroneWayPointFlight::startFlight(void)
{
	std::cout << "startFlight" << std::endl;

	_client.moveOnPathAsync(
		_path,
		_velocity,
		Utils::max<float>(),
		DrivetrainType::ForwardOnly,
		YawMode(false, 0.0)
		//-1,
		//0
	)->waitOnLastTask();

	std::cout << "Go home" << std::endl;
	_client.goHomeAsync()->waitOnLastTask();
	//std::cout << "Land" << std::endl;
	//_client.landAsync()->waitOnLastTask();
}

int main(void) 
{
	DroneWayPointFlight drone_waypoint_flight;
	drone_waypoint_flight.startFlight();

	return 0;
}
