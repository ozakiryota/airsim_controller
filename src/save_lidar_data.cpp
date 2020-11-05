#include "api/RpcLibClientBase.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

#include"cnpy.h"

class SaveLidarData{
	private:
		/*client*/
		msr::airlib::RpcLibClientBase _client;
		/*pc*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr _pc {new pcl::PointCloud<pcl::PointXYZ>};
		/*parameter*/
		const std::string _save_pcd_path = "/home/airsim_ws/airsim_controller/save/tmp/tmp.pcd";
		const std::string _save_npy_path = "/home/airsim_ws/airsim_controller/save/tmp/tmp.npy";
		const int _num_rings = 32;
		const int _points_per_ring = 1812;
		const double _fov_upper_deg = 15;
		const double _fov_lower_deg = -25;

	public:
		SaveLidarData();
		void process(void);
		void inputDataToPc(msr::airlib::LidarData lidar_data);
		void pcNedToNeu(void);
		void visualization(void);
		void savePCD(void);
		void saveNPY(void);
};

SaveLidarData::SaveLidarData()
{
	std::cout << "----- save_lidar_data -----" << std::endl;
	/*initialize*/
	_client.confirmConnection();
}

void SaveLidarData::process(void)
{
	msr::airlib::LidarData lidar_data = _client.getLidarData("");
	inputDataToPc(lidar_data);
	pcNedToNeu();
	// visualization();
	savePCD();
	saveNPY();
}

void SaveLidarData::inputDataToPc(msr::airlib::LidarData lidar_data)
{
	/*input*/
	for(size_t i=0; i<lidar_data.point_cloud.size(); i+=3){
		std::cout
			<< "i = " << i << "~" << i+2 << ": "
			"x = " << lidar_data.point_cloud[i] << ", "
			"y = " << lidar_data.point_cloud[i+1] << ", "
			"z = " << lidar_data.point_cloud[i+2] << std::endl;
		pcl::PointXYZ tmp;
		tmp.x = lidar_data.point_cloud[i];
		tmp.y = lidar_data.point_cloud[i+1];
		tmp.z = lidar_data.point_cloud[i+2];
		_pc->points.push_back(tmp);
	}
	/*width x height*/
	_pc->width = _pc->points.size();
	_pc->height = 1;
	/*print*/
	std::cout << "_pc->points.size() = " << _pc->points.size() << std::endl;
}

void SaveLidarData::pcNedToNeu(void)
{
	std::cout << "NED -> NEU" << std::endl;
	for(size_t i=0; i<_pc->points.size(); ++i){
		_pc->points[i].y *= -1;
		_pc->points[i].z *= -1;
	}
}

void SaveLidarData::visualization(void)
{
	pcl::visualization::CloudViewer viewer("save_lidar_data");
	viewer.showCloud(_pc);
	while(!viewer.wasStopped()){
	}
}

void SaveLidarData::savePCD(void)
{
	pcl::io::savePCDFileASCII(_save_pcd_path, *_pc);
	std::cout << "Saved: " << _save_pcd_path << std::endl;
}

void SaveLidarData::saveNPY(void)
{
	/*resolution*/
	double angle_h_resolution = (_fov_upper_deg - _fov_lower_deg)/180.0*M_PI/(double)(_num_rings - 1);
	double angle_w_resolution = 2*M_PI/(double)_points_per_ring;
	/*initialize*/
	std::vector<double> mat(_num_rings*_points_per_ring, 0.0);
	/*input*/
	for(size_t i=0; i<_pc->points.size(); ++i){
		/*row*/
		double angle_h = atan2(_pc->points[i].z, sqrt(_pc->points[i].x*_pc->points[i].x + _pc->points[i].y*_pc->points[i].y));
		int row = (_fov_upper_deg/180.0*M_PI - angle_h)/angle_h_resolution;
		/*col*/
		double angle_w = atan2(_pc->points[i].y, _pc->points[i].x);
		int col = (_points_per_ring - 1) - (int)((angle_w + M_PI)/angle_w_resolution);
		/*depth*/
		double depth = sqrt(_pc->points[i].x*_pc->points[i].x + _pc->points[i].y*_pc->points[i].y);
		/*input*/
		mat[row*_points_per_ring + col] = depth;
	}
	/*save*/
	cnpy::npy_save(_save_npy_path, &mat[0], {_num_rings, _points_per_ring}, "w");
	std::cout << "Saved: " << _save_npy_path << std::endl;
	/*print*/
	//std::cout << "mat: " << std::endl;
	//for(int row=0; row<_num_rings; ++row){
	//	for(int col=0; col<_points_per_ring; ++col){
	//		std::cout << mat[row*_num_rings + col] << " ";
	//	}
	//	std::cout << std::endl;
	//}
}

int main(void) 
{
	SaveLidarData save_lidar_data;
	save_lidar_data.process();

	return 0;
}
