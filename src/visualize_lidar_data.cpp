#include "api/RpcLibClientBase.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class VisualizeLidarData{
	private:
		/*client*/
		msr::airlib::RpcLibClientBase _client;
		/*pc*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr _pc {new pcl::PointCloud<pcl::PointXYZ>};
		/*list*/
		/*csv*/
		std::ofstream _csvfile;
		/*parameter*/

	public:
		VisualizeLidarData();
		void getData(void);
		void visualization(void);
};

VisualizeLidarData::VisualizeLidarData()
{
	std::cout << "----- visualize_lidar_data -----" << std::endl;
}

void VisualizeLidarData::getData(void)
{
	msr::airlib::LidarData lidar_data = _client.getLidarData("");
	for(int i=0; i<lidar_data.point_cloud.size(); i+=3){
		std::cout 
			<< i << ": " << lidar_data.point_cloud[i] << ", "
			<< i+1 << ": " << lidar_data.point_cloud[i+1] << ", "
			<< i+2 << ": " << lidar_data.point_cloud[i+2] << std::endl;
		pcl::PointXYZ tmp;
		tmp.x = lidar_data.point_cloud[i];
		tmp.y = lidar_data.point_cloud[i+1];
		tmp.z = lidar_data.point_cloud[i+2];
		_pc.points.push_back(tmp);
	}
	visualization();
}

void VisualizeLidarData::visualization(void)
{
	pcl::visualization::CloudViewer viewer("visualize_lidar_data");
	viewer.showCloud(_pc);
	while(!viewer.wasStopped()){
	}
}

int main(void) 
{
	VisualizeLidarData visualize_lidar_data;
	visualize_lidar_data.getData();

	return 0;
}
