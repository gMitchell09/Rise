#include "Urg_Helper.Mock.h"
#include "SerialClass.Mock.h"

Urg_Helper_Mock::Urg_Helper_Mock()
{
	this->fakeUrg = new FakeUrg();
}

bool Urg_Helper_Mock::ConnectToUrg()
{
	try
	{
		Serial_Mock *s = new Serial_Mock(std::string("C:\\Users\\George\\Documents\\GitHub\\Rise\\Simulation\\imu.txt"));
		_imu = new Common::IMU(s);
		this->spawnIMUThread();
	}
	catch (...)
	{
		return false;
	}
	return true;
}

void Urg_Helper_Mock::GetScanFromUrg()
{
	std::vector<long> data;
	long timestamp;
	if (!fakeUrg->get_distance(data, &timestamp))
	{
		return;
	}

	Common::Quaternion qt = _imu->findTimestamp(timestamp);
	if (qt.Q0 == -1 && qt.Q1 == -1 && qt.Q2 == -1 && qt.Q3 == -1) return;
	double rotation = qt.yaw();

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

Urg_Helper_Mock::~Urg_Helper_Mock()
{
	delete fakeUrg;
}