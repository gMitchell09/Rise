// Written by:		David Brown
// Final Update:	April 30th 2014
// Description:		This program combines two libraries. It connects to the LIDAR and the Arduino through a serial connection. 
//					It then gets the data from both and attempts to combine them into a 3D point cloud which is stored in a point cloud library point cloud
//

#include "stdafx.h"
#include "Urg_Helper.h"
#include "Quaternion_Common.h"
#include "Point_Common.h"

#include <thread>
#include <mutex>
#include <string>

#include <pcl/common/common_headers.h>

#define LIDAR_IMU_TIME_THRESHOLD 100
#define ABS(x) ((x < 0) ? (-x) : (x))

//Contructor that intiates both a the pointcloud and the drvier for the lidar
Urg_Helper::Urg_Helper() : 
	_imuThread(NULL), 
	_visualizer(new pcl::visualization::PCLVisualizer("Cloud"))
{
	_updateMutex = new std::mutex();
	Urg_Helper::cloud = new pcl::PointCloud <pcl::PointXYZ>();
	Urg_Helper::urg = new qrk::Urg_driver();
}
//Decontructor that deletes the cloud closes the Lidar connection and deletes the object
Urg_Helper::~Urg_Helper()
{
	delete cloud;
	Urg_Helper::urg->close();
	delete urg;

	if (_imuThread != NULL && _imuThread->joinable())
		_imuThread->join();

	delete _imu;
}
//This function takes each point input and cnoverts it into a 3D point. using the scanNo from the URG a radius and the phi angle
pcl::PointXYZ* Urg_Helper::CreatePoint(int ScanNo, int radius, float angle, bool degrees)
{
	//Creates a pcl point object to store the 3D data.
	pcl::PointXYZ* temp = new pcl::PointXYZ();
	// gets the theta angle from the urg function that converts the scan number into an angle in radians.
	double theta = urg->index2rad(ScanNo);
	//Converts degrees to radians
	double realAngle;
	if (degrees)
		realAngle = (double) (angle * 3.14159265359)/180;
	else
		realAngle = angle;
	// These formaluas are taken directly from spherical coordinates calculations. 
	temp->x = static_cast<float>(cos(realAngle) * sin(theta) * radius);
	temp->y = static_cast<float>(sin(realAngle) * sin(theta) * radius);
	temp->z = static_cast<float>(radius * cos(theta));
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
		_imu = new Common::IMU(std::string("\\\\.\\COM19"));
		this->spawnIMUThread();

		urg->start_measurement(qrk::Urg_driver::Distance);
		Sleep(2000);
	}
	catch (...)
	{
		return false;
	}
	return true;
}

void Urg_Helper::GetScanFromUrg()
{
	std::vector<long> data;
	long timestamp;
	if (!urg->get_distance(data, &timestamp))
	{
		return;
	}
	Common::Quaternion qt = _imu->findTimestamp(timestamp);
	if (qt.Q0 == -1 && qt.Q1 == -1 && qt.Q2 == -1 && qt.Q3 == -1) return;
	double rotation = qt.yaw();

	/*angle = time_stamp/1000.0 * 360;*/
	for (int i = 0; i < data.size(); i++)
	{
		pcl::PointXYZ *tempPoint = CreatePoint(i, data[i], rotation, false);
		cloud->push_back(*tempPoint);
		delete tempPoint;
	}

	_updateMutex->lock();
	_updateCloud = true;
	_updateMutex->unlock();
}

void Urg_Helper::spawnIMUThread()
{
	_imuThread = _imu->make_thread();
}

//Calcutes the phi angle from the step passed by the arduino
double Urg_Helper::CalculatePhiAngle(int step)
{
	// Since the total number of steps in a roation is 2048 divide by 2047 and multiply by 360 to get the angle
	double phiAngle = (((double) step) / (double) 2047) * 360;
	return phiAngle;
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
	  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	  std::cout << "Genarating example point clouds.\n\n";
	  // We're going to make an ellipse extruded along the z-axis. The colour for
	  // the XYZRGB cloud will gradually go from red to green to blue.
	  uint8_t r(255), g(15), b(15);
	  for (float z(-1.0); z <= 1.0; z += 0.05)
	  {
		for (float angle(0.0); angle <= 360.0; angle += 5.0)
		{
			pcl::PointXYZ basic_point;
			basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
			basic_point.y = sinf (pcl::deg2rad(angle));
			basic_point.z = z;
			basic_cloud_ptr->points.push_back(basic_point);

			pcl::PointXYZRGB point;
			point.x = basic_point.x;
			point.y = basic_point.y;
			point.z = basic_point.z;
			uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
					static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
			point.rgb = *reinterpret_cast<float*>(&rgb);
			point_cloud_ptr->points.push_back (point);
		}
		if (z < 0.0)
		{
			r -= 12;
			g += 12;
		}
		else
		{
			g -= 12;
			b += 12;
		}
	  }
	  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
	  basic_cloud_ptr->height = 1;
	  point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
	  point_cloud_ptr->height = 1;

	  /* end */

	_visualizer->setBackgroundColor(0.0, 0.0, 0.0);
	_visualizer->addPointCloud<pcl::PointXYZ> (basic_cloud_ptr, "input cloud");
	_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input cloud");
	_visualizer->addCoordinateSystem(1.0);
	_visualizer->initCameraParameters();

	while (!_visualizer->wasStopped())
	{
		_visualizer->spinOnce(100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	return true;
}
