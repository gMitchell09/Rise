// Written by:		David Brown
// Final Update:	April 30th 2014
// Description:		This program combines two libraries. It connects to the LIDAR and the Arduino through a serial connection. 
//					It then gets the data from both and attempts to combine them into a 3D point cloud which is stored in a point cloud library point cloud
//

// Updated by:		George Mitchell
// Changelog:		Completely cleaned-up code and dependencies, very little original code remains.
//						History is viewable from github: github.com/gMitchell09/Rise

#include "stdafx.h"
#include "Urg_Helper.h"
#include "Quaternion_Common.h"
#include "Point_Common.h"
#include "CloudMeshAdapter.h"
#include "TravelGrid.h"

#include <thread>
#include <mutex>
#include <string>
#include <cmath>

#define isnan(x) _isnan(x)
#define isfinite(x) _finite(x)

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <boost/interprocess/mapped_region.hpp>

#define ABS(x) ((x < 0) ? (-x) : (x))

//Contructor that intiates both a the pointcloud and the drvier for the lidar
Urg_Helper::Urg_Helper() : 
	_visualizer(new pcl::visualization::PCLVisualizer("Cloud")),
	cloud(new pcl::PointCloud <pcl::PointXYZRGB>()),
	urg(std::unique_ptr<qrk::Urg_driver>(new qrk::Urg_driver())),
	_updateMutex(std::unique_ptr<std::mutex> (new std::mutex()))
{}

//Decontructor that deletes the cloud closes the Lidar connection and deletes the object
Urg_Helper::~Urg_Helper()
{
	_visualizer->close();

	this->urg->close();
}
//This function takes each point input and cnoverts it into a 3D point. using the scanNo from the URG a radius and the phi angle
pcl::PointXYZRGB Urg_Helper::CreatePoint(int ScanNo, int radius, float angle, Common::PointXYZ roverPos)
{
	//Creates a pcl point object to store the 3D data.
	pcl::PointXYZRGB temp;
	// gets the theta angle from the urg function that converts the scan number into an angle in radians.
	double theta = urg->index2rad(ScanNo);

	// First convert polar coordinates (r, theta coming straight from LIDAR)
	pcl::PointXYZRGB lidarCart;
	lidarCart.x = 0;
	lidarCart.y = static_cast<float>(cos(theta) * radius);
	lidarCart.z = static_cast<float>(sin(theta) * radius);

	// Rotate these values about the Z axis from the IMU yaw value
	temp.x = static_cast<float>(lidarCart.x * cos(angle) + lidarCart.y * sin(angle));
	temp.y = static_cast<float>(-lidarCart.x * sin(angle) + lidarCart.y * cos(angle));
	temp.z = static_cast<float>(lidarCart.z);

	temp.x += roverPos.x;
	temp.y += roverPos.y;
	temp.z += roverPos.z;

	temp.r = 255;
	temp.g = 255;
	temp.b = 255;

	return temp;
}

//This method connects to the urg and arduino please update each port for the connection
bool Urg_Helper::ConnectToUrg()
{
	//If having trouble connecting change this port to the correct one for the LiDar
	if (!urg->open("COM24", 115200, qrk::Urg_driver::Serial))
	{
		std::cout << "Could not connect to URG." << endl;
		return false;
	}
	//Goes from 0 to 1080 wich is -135 degrees to +135
	urg->set_scanning_parameter(0, 1080, 0);
	//This port is for the arduino. Leave in the backslashes and periods.
	try
	{
		std::shared_ptr<Serial> rotSerial = std::shared_ptr<Serial> (new Serial(std::string("\\\\.\\COM19")));
		_rotImu = std::shared_ptr<Common::IMU>(new Common::IMU(rotSerial));

		std::shared_ptr<Serial> posSerial = std::shared_ptr<Serial> (new Serial(std::string("\\\\.\\COMXX")));
		_posIMU = std::shared_ptr<Common::IMU>(new Common::IMU(posSerial));

		urg->start_measurement(qrk::Urg_driver::Distance);
		Sleep(2000);
	}
	catch (...)
	{
		return false;
	}
	return true;
}

bool Urg_Helper::GetScanFromUrg()
{
	std::vector<long> data;
	long timestamp;
	if (!urg->get_distance(data, &timestamp))
	{
		return false;
	}

	Common::Quaternion qt = _rotImu->findTimestamp(timestamp);
	if (isnan(qt.x) || isnan(qt.y) || isnan(qt.z) || isnan(qt.w)) return false;
	double rotation = qt.pitch();

	Common::Quaternion pos = _posIMU->findTimestamp(timestamp);
	if (isnan(pos.x) || isnan(pos.y) || isnan(pos.z)) return false;

	for (int i = 0; i < data.size(); i++)
	{
		if (!isfinite(data[i])) continue;
		pcl::PointXYZRGB tempPoint = CreatePoint(i, data[i], rotation, pos.GetPoint());
		cloud->push_back(tempPoint);
	}

	_updateMutex->lock();
	_updateCloud = true;
	_updateMutex->unlock();

	return true;
}

//This function starts the visualization from the point cloud library. Can be modified according to point cloud library
// for more functinoaliy like direction or colored points.
bool Urg_Helper::StartCloudVisualization()
{
	return this->StartPCLVisualizer();

	if (!cloud->empty())
	{
		pcl::visualization::CloudViewer cloudviewer("Cloud");
		cloudviewer.showCloud(pcl::PointCloud <pcl::PointXYZRGB>::Ptr(cloud));
		// Visualization stops after a short time, this loop keeps it going.
		while (!cloudviewer.wasStopped ())
		{
			GetScanFromUrg();
		}
		return true;
	}
	return false;
}

bool Urg_Helper::StartPCLVisualizer()
{
	/* Remove when doing the real-deal(tm) */
	  std::cout << "Genarating example point clouds.\n\n";

	//while( this->GetScanFromUrg() );
	//while( this->GetScanFromUrg() );
	//while( this->GetScanFromUrg() );
	//while( this->GetScanFromUrg() );
	//while( this->GetScanFromUrg() );
	this->GetScanFromUrg();

	//_visualizer->addPolygonMesh(*CloudMeshAdapter::GetMeshFromCloud(cloud->makeShared()));
	
	//std::cout << "Polygon mesh generated" << std::endl;

	/*Eigen::Affine3f t;
	t.scale(Eigen::Vector3f(0.01, 0.01, 0.01));
	_visualizer->addCoordinateSystem(1.0, t);*/
	//_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "input cloud");
	//_visualizer->addPlane(CloudMeshAdapter::GetPlanesFromCloud(cloud->makeShared()), "wall_0");
	//_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "wall_0");

	//cloud = CloudMeshAdapter::PassThroughFilter(cloud->makeShared());
	//cloud = CloudMeshAdapter::StatisticOutlierRemovalFilter(cloud->makeShared());
	//std::vector<pcl::ModelCoefficients> planes = CloudMeshAdapter::PlaneDetection(cloud->makeShared());

	//_visualizer->addPointCloud<pcl::PointXYZRGB> (cloud->makeShared(), "input cloud");

	//for (int x = -1000; x < 1000; x+=100)
	//{
	//	for (int y = -1000; y < 1000; y+=100)
	//	{
	//		pcl::PointXYZRGBRGB a(128, 128, 0);
	//		pcl::PointXYZRGBRGB b(128, 128, 0);
	//		a.x = b.x = x;
	//		a.y = b.y = y;
	//		a.z = b.z = 0;
	//		std::string id = "Line: " + std::to_string(x) + "," + std::to_string(y);
	//		_visualizer->addLine<pcl::PointXYZRGBRGB>(a, b, id);
	//		_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 30, id);
	//	}
	//}

	_visualizer->addCoordinateSystem(1000.0);
	_visualizer->setBackgroundColor(0.0, 0.0, 0.0);
	_visualizer->initCameraParameters();
	_visualizer->setCameraPosition(5000, -5000, 5000, 0, 0, 5000);

	boost::function <void(const pcl::visualization::KeyboardEvent&)> f =
		boost::bind(&Urg_Helper::keyPress, this, _1);

	_visualizer->registerKeyboardCallback(f);
	while (!_visualizer->wasStopped())
	{
		if (!_visualizer->updatePointCloud(pcl::PointCloud <pcl::PointXYZRGB>::ConstPtr(cloud), "input cloud"))
		{
			_visualizer->addPointCloud<pcl::PointXYZRGB> (pcl::PointCloud <pcl::PointXYZRGB>::Ptr(cloud), "input cloud");
			_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input cloud");
		}

		_visualizer->spinOnce(1);
		this->GetScanFromUrg();
	}

	return true;
}

void Urg_Helper::keyPress (const pcl::visualization::KeyboardEvent &ev)
{
	std::cout << "Key: " << ev.getKeySym() << " : " << ev.getKeyCode() << std::endl;
	std::cout << "Mod: " << ev.isAltPressed() << ", " << ev.isCtrlPressed() << ", " << ev.isShiftPressed() << std::endl;

	Eigen::Vector3f xAxis(1, 0, 0);
	Eigen::Vector3f yAxis(0, 1, 0);
	Eigen::Vector3f zAxis(0, 0, 1);

	if (ev.getKeyCode() == 's')
	{
		std::unique_ptr<pcl::FileWriter> fw (new pcl::PLYWriter());
		//std::unique_ptr<pcl::FileWriter> fw (new pcl::PCDWriter());

		this->ExportPointCloud("C:\\Users\\George\\Desktop\\Test.ply", std::move(fw));
		std::cout << "Saved!" << std::endl;
	}
	else if (ev.getKeyCode() == 'b' && ev.keyDown())
	{
		CloudMeshAdapter::PlaneDetection(cloud, xAxis);
	}
	else if (ev.getKeyCode() == 'n' && ev.keyDown())
	{
		CloudMeshAdapter::PlaneDetection(cloud, yAxis);
	}
	else if (ev.getKeyCode() == 'm' && ev.keyDown())
	{
		CloudMeshAdapter::PlaneDetection(cloud, zAxis);
	}
	else if (ev.getKeyCode() == 'p' && ev.keyDown())
	{
		TravelGrid tg;
		tg.setInputCloud(this->cloud);
		
		pcl::IndicesPtr indices = tg.points_in_range(0, 1000, 0, 1000);
		TravelGrid::Cell::CellTypes cellType = tg.classify_cell(indices);

		uint8_t r = 0;
		uint8_t g = 0;
		uint8_t b = 255;

		switch (cellType)
		{
		case TravelGrid::Cell::CellTypes::kUndefined:
			break;
		case TravelGrid::Cell::CellTypes::kFloor:
			r = b = 0;
			g = 255;
			break;
		case TravelGrid::Cell::CellTypes::kWall:
			r = 255;
			g = b = 0;
			break;
		case TravelGrid::Cell::CellTypes::kStairs:
			r = g = 255;
			b = 0;
			break;
		}

		for (auto itr = indices->begin(); itr != indices->end(); ++itr)
		{
			this->cloud->points[*itr].r = r;
			this->cloud->points[*itr].g = g;
			this->cloud->points[*itr].b = b;
		}
	}
}

bool Urg_Helper::ExportPointCloud(std::string filename, std::unique_ptr<pcl::FileWriter> fw)
{
	//return false;
	return fw->write(filename, *cloud) > 0;
}

bool Urg_Helper::ImportPointCloud(std::string filename, std::unique_ptr<pcl::FileReader> fr)
{
	return fr->read(filename, *cloud) > 0;
}