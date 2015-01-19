// Urg_Helper.h

#pragma once
#include "Stdafx.h"
#include "SerialClass.h"
#include "IMU.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <memory> // for std::unique_ptr

class Urg_Helper
{
public:
	// TODO: Add your methods for this class here.
	bool ConnectToUrg();
	void GetScanFromUrg();
	void spawnIMUThread();

	bool StartCloudVisualization();
	bool StartPCLVisualizer();

	Urg_Helper();
	~Urg_Helper();

private:
	pcl::PointCloud <pcl::PointXYZ>* cloud;
	pcl::PointXYZ *CreatePoint(int ScanNo, int radius, float angle, bool degrees);
	qrk::Urg_driver *urg;

	Common::IMU *_imu;
	Serial *_serial;
	std::thread *_imuThread;

	double CalculatePhiAngle(int step);

	bool _updateCloud;
	std::mutex* _updateMutex;

	std::unique_ptr<pcl::visualization::PCLVisualizer> _visualizer;
};
