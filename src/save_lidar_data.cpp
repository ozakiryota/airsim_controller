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
		/*path*/
		std::string _save_path = "/home/airsim_ws/airsim_controller/save/tmp/tmp.pcd";

	public:
		SaveLidarData();
		void getData(void);
		void pcAdjustWH(void);
		void pcNedToNeu(void);
		void visualization(void);
		void save(void);
};

SaveLidarData::SaveLidarData()
{
	std::cout << "----- save_lidar_data -----" << std::endl;
	_client.confirmConnection();
}

void SaveLidarData::getData(void)
{
	msr::airlib::LidarData lidar_data = _client.getLidarData("");
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
	pcAdjustWH();
	pcNedToNeu();
	// visualization();
	save();
}

void SaveLidarData::pcAdjustWH(void)
{
	_pc->width = _pc->points.size();
	_pc->height = 1;
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

void SaveLidarData::save(void)
{
	pcl::io::savePCDFileASCII(_save_path, *_pc);
	std::cout << "The point cloud was saved as " << _save_path << std::endl;
}

int main(void) 
{
	SaveLidarData save_lidar_data;
	save_lidar_data.getData();

	return 0;
}
