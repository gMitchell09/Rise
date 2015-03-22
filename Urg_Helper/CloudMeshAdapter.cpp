#include "CloudMeshAdapter.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h> 

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

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

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudMeshAdapter::PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

	double passFilterXMin = -30000, passFilterXMax = 30000;
	double passFilterYMin = -30000, passFilterYMax = 30000;
	pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr outCloudInt(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (passFilterXMin, passFilterXMax);
	pass.filter(*outCloud);

	pass.setInputCloud(outCloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(passFilterYMin, passFilterYMax);
	pass.filter(*outCloudInt);

	pass.setInputCloud(outCloudInt);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(passFilterYMin, passFilterYMax);
	pass.filter(*outCloud);

	return outCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudMeshAdapter::StatisticOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	double outlierMeanK = 10;
	double outlierStddevMulThresh = 10;
	pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(outlierMeanK);
	sor.setStddevMulThresh(outlierStddevMulThresh);
	sor.filter(*outCloud);

	return outCloud;
}

std::vector<pcl::ModelCoefficients> CloudMeshAdapter::UnorganizedPlaneDetection(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	std::vector<pcl::ModelCoefficients> coeff;
	//// First we will apply a VoxelGrid filter to our point cloud to eliminate the reduntant points
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
	//pcl::VoxelGrid<pcl::PointXYZ> sor;
	//sor.setInputCloud(cloud);
	//sor.setLeafSize(10, 10, 10); // set leaf size to 1cm
	//sor.filter (*cloud_filtered);


	//pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients());
	//pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

	//// Create segmentation object
	//pcl::SACSegmentation<pcl::PointXYZ> seg;
	//seg.setOptimizeCoefficients(true);
	//seg.setModelType (pcl::SACMODEL_PLANE);
	//seg.setMethodType(pcl::SAC_RANSAC);
	//seg.setMaxIterations(1000);
	//seg.setDistanceThreshold(10);

	//pcl::ExtractIndices<pcl::PointXYZ> extract;

	//int i=0, nr_points = (int)cloud_filtered->points.size();
	//while (cloud_filtered->points.size() > 0.3 * nr_points)
	//{
	//	seg.setInputCloud (cloud_filtered);
	//	seg.segment (*inliers, *coeff);
	//	if (inliers->indices.size() == 0)
	//	{
	//		std::cerr << "Could not estimage a planar model for the given dataset" << std::endl;
	//		break;
	//	}

	//	extract.setInputCloud (cloud_filtered);
	//	extract.setIndices (inliers);
	//	extract.setNegative (false);
	//	extract.filter (*cloud_p);
	//	std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

	//	extract.setNegative(true);
	//	extract.filter (*cloud_f);
	//	cloud_filtered.swap (cloud_f);

	//	i++;
	//}

	return coeff;
}

std::vector<pcl::ModelCoefficients> CloudMeshAdapter::PlaneDetection(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	std::cout << "Plane detection" << std::endl;
	//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	//pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	//tree->setInputCloud (cloud);
	//n.setInputCloud (cloud);
	//n.setSearchMethod (tree);
	//n.setKSearch (20);
	//n.compute (*normals);

	pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ>::Ptr model_p(
		new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ>(cloud));
	Eigen::Vector3f xAxis(0, 1, 1);
	Eigen::Vector3f yAxis(1, 0, 1);
	Eigen::Vector3f zAxis(1, 1, 0);
	Eigen::Vector3f nxAxis(0, -1, -1);
	Eigen::Vector3f nyAxis(-1, 0, -1);
	Eigen::Vector3f nzAxis(-1, -1, 0);

	model_p->setAxis(xAxis);
	model_p->setEpsAngle(pcl::deg2rad(15.0));
	
	std::vector <int> inliers;
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
	ransac.setDistanceThreshold(50);
	ransac.computeModel(2);
	ransac.getInliers(inliers);

	Eigen::VectorXf mCoeffs;
	ransac.getModelCoefficients(mCoeffs);

	std::cout << mCoeffs.size() << std::endl;
	std::cout << "Inliers size: " << inliers.size() << std::endl;

	// TODO: Get smallest and largest inliers for each axis and return corners for plane

	for (int i = 0; i < mCoeffs.size(); ++i)
	{
		std::cout << mCoeffs[i] << ", ";
	}

	std::cout << std::endl;

	std::vector<pcl::ModelCoefficients> coeff;
	return coeff;
}

void CloudMeshAdapter::FlattenCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud3d, pcl::PointCloud<pcl::PointXYZL>::Ptr grid2d, size_t cellWidth)
{
	// steps:
	//    Iterate over grid with cells ranging from (x, y) to (x + cellWidth, y + cellWidth) and grab all points with any Z-value.
	//	  Convert this to a PointXYZL with Z = width, L = #points
	// profit.
}

