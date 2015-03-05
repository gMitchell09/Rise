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

#include <thread>
#include <mutex>
#include <string>
#include <cmath>
#define isnan(x) _isnan(x)

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <boost/interprocess/mapped_region.hpp>

#define ABS(x) ((x < 0) ? (-x) : (x))

//Contructor that intiates both a the pointcloud and the drvier for the lidar
Urg_Helper::Urg_Helper() : 
	_visualizer(new pcl::visualization::PCLVisualizer("Cloud")),
	cloud(new pcl::PointCloud <pcl::PointXYZ>()),
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
pcl::PointXYZ Urg_Helper::CreatePoint(int ScanNo, int radius, float angle, Common::PointXYZ roverPos)
{
	//Creates a pcl point object to store the 3D data.
	pcl::PointXYZ temp;
	// gets the theta angle from the urg function that converts the scan number into an angle in radians.
	double theta = urg->index2rad(ScanNo);

	// First convert polar coordinates (r, theta coming straight from LIDAR)
	pcl::PointXYZ lidarCart;
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
	double rotation = qt.yaw();

	Common::Quaternion pos = _posIMU->findTimestamp(timestamp);
	if (isnan(pos.x) || isnan(pos.y) || isnan(pos.z)) return false;

	for (int i = 0; i < data.size(); i++)
	{
		pcl::PointXYZ tempPoint = CreatePoint(i, data[i], rotation, pos.GetPoint());
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
		cloudviewer.showCloud(pcl::PointCloud <pcl::PointXYZ>::Ptr(cloud));
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

	//_visualizer->addPointCloud<pcl::PointXYZ> (cloud->makeShared(), "input cloud");
	
	_visualizer->addCoordinateSystem(1000.0);
	_visualizer->setBackgroundColor(0.0, 0.0, 0.0);
	_visualizer->initCameraParameters();
	_visualizer->setCameraPosition(5000, -5000, 5000, 0, 0, 5000);

	boost::function <void(const pcl::visualization::KeyboardEvent&)> f =
		boost::bind(&Urg_Helper::keyPress, this, _1);

	_visualizer->registerKeyboardCallback(f);
	while (!_visualizer->wasStopped())
	{
		if (!_visualizer->updatePointCloud(pcl::PointCloud <pcl::PointXYZ>::ConstPtr(cloud), "input cloud"))
		{
			_visualizer->addPointCloud<pcl::PointXYZ> (pcl::PointCloud <pcl::PointXYZ>::Ptr(cloud), "input cloud");
			_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input cloud");
		}

		_visualizer->spinOnce(1);
		//std::this_thread::sleep_for(std::chrono::milliseconds(100));
		this->GetScanFromUrg();

		{
			_updateMutex->lock();
			if (_updateCloud)
			{
				//_visualizer->updatePointCloud(pcl::PointCloud <pcl::PointXYZ>::Ptr(cloud), "input cloud");
				_updateCloud = false;
			}
			_updateMutex->unlock();
		}
	}

	return true;
}

void Urg_Helper::keyPress (const pcl::visualization::KeyboardEvent &ev)
{
	std::cout << "Key: " << ev.getKeySym() << " : " << ev.getKeyCode() << std::endl;
	std::cout << "Mod: " << ev.isAltPressed() << ", " << ev.isCtrlPressed() << ", " << ev.isShiftPressed() << std::endl;
	if (ev.getKeyCode() == 's')
	{
		std::unique_ptr<pcl::FileWriter> fw (new pcl::PLYWriter());
		//std::unique_ptr<pcl::FileWriter> fw (new pcl::PCDWriter());

		this->ExportPointCloud("C:\\Users\\George\\Desktop\\Test.ply", std::move(fw));
		std::cout << "Saved!" << std::endl;
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