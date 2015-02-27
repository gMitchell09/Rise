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

	static pcl::PolygonMesh::ConstPtr GetMeshFromCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
	static pcl::ModelCoefficients GetPlanesFromCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
	
	static pcl::PointCloud<pcl::PointXYZ>::Ptr PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	static pcl::PointCloud<pcl::PointXYZ>::Ptr StatisticOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	static std::vector<pcl::ModelCoefficients> PlaneDetection(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
};

