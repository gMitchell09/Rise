// Urg_Helper.h

#pragma once
#include "Stdafx.h"
#include "SerialClass.h"
#include "IMU.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/file_io.h>

#include <memory> // for std::unique_ptr
#include <thread>
#include <mutex>

class Urg_Helper
{
public:
	MOCKABLE bool ConnectToUrg();
	MOCKABLE bool GetScanFromUrg();

	bool StartCloudVisualization();
	bool StartPCLVisualizer();
	bool ExportPointCloud(std::string filename, std::unique_ptr<pcl::FileWriter> fw);
	bool ImportPointCloud(std::string filename, std::unique_ptr<pcl::FileReader> fr);

	Urg_Helper();
	~Urg_Helper();

protected:
	pcl::PointCloud <pcl::PointXYZ> *cloud;
	pcl::PointXYZ CreatePoint(int ScanNo, int radius, float angle, bool degrees);
	qrk::Urg_driver *urg;

	Common::IMU *_imu;
	std::thread *_imuThread;

	bool _updateCloud;
	std::mutex* _updateMutex;

	std::unique_ptr<pcl::visualization::PCLVisualizer> _visualizer;
};
