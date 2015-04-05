#pragma once

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

class CloudMeshAdapter
{
public:
	// We don't want to be able to instantiate any of these classes... yet...
	//CloudMeshAdapter() = delete;
	//~CloudMeshAdapter() = delete;

	static pcl::PolygonMesh::ConstPtr GetMeshFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	static pcl::ModelCoefficients GetPlanesFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr PassThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr StatisticOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	std::vector<pcl::ModelCoefficients> 
	static CloudMeshAdapter::PlaneDetection(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		Eigen::Vector3f axis);
	static void FlattenCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud3d, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr grid2d, size_t cellWidth);
	static std::vector<pcl::ModelCoefficients> UnorganizedPlaneDetection(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

	static void VoxelGridFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
};

