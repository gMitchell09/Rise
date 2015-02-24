#include "Urg_Helper.Mock.h"
#include "SerialClass.Mock.h"

Urg_Helper_Mock::Urg_Helper_Mock()
{
	this->urg = new FakeUrg();
}

Urg_Helper_Mock::~Urg_Helper_Mock()
{
}

bool Urg_Helper_Mock::ConnectToUrg()
{
	std::cout << "Connecting to URG" << std::endl;
	try
	{
		Serial_Mock *s = new Serial_Mock(std::string("C:\\Users\\MitchellGe\\Documents\\GitHub\\Rise\\Simulation\\imu9.txt"));
		_imu = new Common::IMU(s);
		Sleep(1000); // sleep for 1s so IMU data can be processed...
		if (!urg->open("C:\\Users\\MitchellGe\\Documents\\GitHub\\Rise\\Simulation\\lidar9.txt"))
		{
			std::cout << "Could not open fake urg file" << std::endl;
			return false;
		}
	}
	catch (...)
	{
		std::cout << "Exception caught :(" << std::endl;
		return false;
	}
	return true;
}

bool Urg_Helper_Mock::GetScanFromUrg()
{
	std::cout << "Scanning..." << std::endl;
	std::vector<long> data;
	long timestamp;
	if (!urg->get_distance(data, &timestamp))
	{
		return false;
	}

	Common::Quaternion qt = _imu->findTimestamp(timestamp);
	if (qt.x == -1 && qt.y == -1 && qt.z == -1 && qt.w == -1) return false;
	double rotation = qt.yaw();

	for (int i = 0; i < data.size(); i++)
	{
		pcl::PointXYZ tempPoint = CreatePoint(i, data[i], rotation, false);
		cloud->push_back(tempPoint);
	}

	_updateMutex->lock();
	_updateCloud = true;
	_updateMutex->unlock();

	return true;
}
