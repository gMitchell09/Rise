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
CloudMeshAdapter::GetMeshFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	std::cout << "Inside" << std::endl;
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
	std::cout << "-4" << std::endl;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	std::cout << "-3" << std::endl;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	std::cout << "-2" << std::endl;
	tree->setInputCloud (cloud);
	std::cout << "-1" << std::endl;
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	std::cout << "0" << std::endl;
	n.setKSearch (20);
	n.compute (*normals);

	std::cout << "1" << std::endl;

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

	std::cout << "2" << std::endl;

	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	tree2->setInputCloud(cloud_with_normals);

	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
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

pcl::ModelCoefficients CloudMeshAdapter::GetPlanesFromCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	// Create segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudMeshAdapter::PassThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{

	double passFilterXMin = -30000, passFilterXMax = 30000;
	double passFilterYMin = -30000, passFilterYMax = 30000;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloudInt(new pcl::PointCloud<pcl::PointXYZRGB>());

	pcl::PassThrough<pcl::PointXYZRGB> pass;
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudMeshAdapter::StatisticOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	double outlierMeanK = 10;
	double outlierStddevMulThresh = 10;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(outlierMeanK);
	sor.setStddevMulThresh(outlierStddevMulThresh);
	sor.filter(*outCloud);

	return outCloud;
}

std::vector<pcl::ModelCoefficients> CloudMeshAdapter::UnorganizedPlaneDetection(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	std::vector<pcl::ModelCoefficients> coeff;
	//// First we will apply a VoxelGrid filter to our point cloud to eliminate the reduntant points
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
	//pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	//sor.setInputCloud(cloud);
	//sor.setLeafSize(10, 10, 10); // set leaf size to 1cm
	//sor.filter (*cloud_filtered);


	//pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients());
	//pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

	//// Create segmentation object
	//pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	//seg.setOptimizeCoefficients(true);
	//seg.setModelType (pcl::SACMODEL_PLANE);
	//seg.setMethodType(pcl::SAC_RANSAC);
	//seg.setMaxIterations(1000);
	//seg.setDistanceThreshold(10);

	//pcl::ExtractIndices<pcl::PointXYZRGB> extract;

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

std::vector<pcl::ModelCoefficients> 
	CloudMeshAdapter::PlaneDetection(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		Eigen::Vector3f axis)
{
	std::cout << "Plane detection" << std::endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);


	pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZRGB>::Ptr model_p(
		new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZRGB>(cloud));

	model_p->setAxis(axis);
	model_p->setEpsAngle(pcl::deg2rad(7.5));
	
	std::vector <int> inliers;
	pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_p);
	ransac.setDistanceThreshold(50);
	ransac.setMaxIterations(10);
	ransac.computeModel(2);
	ransac.getInliers(inliers);

	Eigen::VectorXf mCoeffs;
	ransac.getModelCoefficients(mCoeffs);

	std::cout << mCoeffs.size() << std::endl;
	std::cout << "Inliers size: " << inliers.size() << std::endl;

	std::vector<pcl::ModelCoefficients> coeff;

	if (inliers.size() == 0) return coeff;

	// TODO: Get smallest and largest inliers for each axis and return corners for plane

	float min_x = std::numeric_limits<float>::max(), 
		min_y = std::numeric_limits<float>::max(), 
		min_z = std::numeric_limits<float>::max(), 
		max_x = std::numeric_limits<float>::min(), 
		max_y = std::numeric_limits<float>::min(), 
		max_z = std::numeric_limits<float>::min();

	for (auto itr = inliers.begin(); itr != inliers.end(); ++itr)
	{
		pcl::PointXYZRGB pt = cloud->points[*itr];
		if (pt.x < min_x) min_x = pt.x;
		if (pt.x > max_x) max_x = pt.x;
		if (pt.y < min_y) min_y = pt.y;
		if (pt.y > max_y) max_y = pt.y;
		if (pt.z < min_z) min_z = pt.z;
		if (pt.z > max_z) max_z = pt.z;
		cloud->points[*itr].b = 0;
	}

	pcl::PointIndices::Ptr indices (new pcl::PointIndices());
	indices->indices = inliers;

	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setNegative(true);
	extract.setInputCloud(cloud);
	extract.setIndices(indices);
	extract.filter(*cloud_p);
	
	cloud->swap(*cloud_p);

	std::cout << "Limits: " << std::endl;
	std::cout << "X: " << min_x << " - " << max_x << std::endl;
	std::cout << "Y: " << min_y << " - " << max_y << std::endl;
	std::cout << "Z: " << min_z << " - " << max_z << std::endl;

	for (int i = 0; i < mCoeffs.size(); ++i)
	{
		std::cout << mCoeffs[i] << ", ";
	}

	std::cout << std::endl;

	return coeff;
}

void CloudMeshAdapter::FlattenCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud3d, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr grid2d, size_t cellWidth)
{
	// steps:
	//    Iterate over grid with cells ranging from (x, y) to (x + cellWidth, y + cellWidth) and grab all points with any Z-value.
	//	  Convert this to a PointXYZL with Z = width, L = #points
	// profit.
}

void CloudMeshAdapter::VoxelGridFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filt(new pcl::PointCloud<pcl::PointXYZRGB>());

	pcl::VoxelGrid<pcl::PointXYZRGB> v;
	v.setInputCloud(cloud);
	v.setLeafSize(50, 50, 50);
	v.filter(*cloud_filt);
	std::cout << "Stupid check: " << cloud_filt->points.size() << std::endl;
	cloud->swap(*cloud_filt);
	std::cout << "Stupid check: " << cloud_filt->points.size() << std::endl;
	std::cout << "Stupid check: " << cloud->points.size() << std::endl;
}