// Urg_Helper.h

#pragma once
#include "Stdafx.h"
#include "SerialClass.h"
#include "IMU.h"

#include <boost/thread.hpp>

using namespace System;
using namespace System::Collections::Generic;
using namespace System::IO::Ports;
using namespace System::Threading;

namespace Urg_Helper {

	public ref class Urg_Helper
	{
	public:
		// TODO: Add your methods for this class here.
		bool ConnectToUrg();
		List<long> ^ GetDataFromTheUrg(double numberofScans);
		void GetScanFromUrg();
		void spawnIMUThread();

		bool StartCloudVisualization();
		Urg_Helper();
		~Urg_Helper();

	private:
		pcl::PointCloud <pcl::PointXYZ>* cloud;
		pcl::PointXYZ *CreatePoint(int ScanNo, int radius, float angle, bool degrees);
		qrk::Urg_driver *urg;

		Common::IMU *_imu;
		Serial *_serial;
		boost::thread *_imuThread;

		double CalculatePhiAngle(int step);
		

	};
}
