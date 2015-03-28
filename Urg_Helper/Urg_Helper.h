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

	void keyPress (const pcl::visualization::KeyboardEvent &ev);

	Urg_Helper();
	~Urg_Helper();

protected:
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud;
	pcl::PointXYZRGB CreatePoint(int ScanNo, int radius, float angle, Common::PointXYZ roverPos);
	
	std::unique_ptr<qrk::Urg_driver> urg;

	std::shared_ptr<Common::IMU> _rotImu;
	std::shared_ptr<Common::IMU> _posIMU;

	bool _updateCloud;
	std::unique_ptr<std::mutex> _updateMutex;

	std::unique_ptr<pcl::visualization::PCLVisualizer> _visualizer;
};
