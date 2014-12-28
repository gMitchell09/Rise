// Urg_Helper.h

#pragma once
#include "Stdafx.h"
#include "SerialClass.h"
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
		bool StartCloudVisualization();
		Urg_Helper();
		~Urg_Helper();
		

	private:
		pcl::PointCloud <pcl::PointXYZ>* cloud;
		pcl::PointXYZ *CreatePoint(int ScanNo, int radius, float angle, bool degrees);
		qrk::Urg_driver *urg;

		Serial *_serial;

		double CalculatePhiAngle(int step);
		

	};
}
