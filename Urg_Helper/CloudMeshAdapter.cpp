#include "CloudMeshAdapter.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>

pcl::PolygonMesh::ConstPtr 
CloudMeshAdapter::GetMeshFromCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	std::cout << "Inside" << std::endl;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	std::cout << "-4" << std::endl;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	std::cout << "-3" << std::endl;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	std::cout << "-2" << std::endl;
	tree->setInputCloud (cloud);
	std::cout << "-1" << std::endl;
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	std::cout << "0" << std::endl;
	n.setKSearch (20);
	n.compute (*normals);

	std::cout << "1" << std::endl;

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

	std::cout << "2" << std::endl;

	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh());

	std::cout << "3" << std::endl;

	gp3.setSearchRadius(500);

	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI/4);
	gp3.setMinimumAngle(M_PI/18);
	gp3.setMaximumAngle(2*M_PI/3);
	gp3.setNormalConsistency(false);

	std::cout << "4" << std::endl;

	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(*triangles);

	std::cout << "5" << std::endl;

	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	std::cout << "6" << std::endl;
	return pcl::PolygonMesh::ConstPtr(triangles);
}

pcl::ModelCoefficients CloudMeshAdapter::GetPlanesFromCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	// Create segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// optional
	seg.setOptimizeCoefficients(true);
	// mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(100);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coeff);

	if (inliers->indices.size() == 0)
	{
		std::cout << "Fuck." << std::endl;
		PCL_ERROR("Could not estimate planar model for dataset");
		return pcl::ModelCoefficients();
	}

	return *coeff;
}