#include "Urg_Helper.Mock.h"
#include "SerialClass.Mock.h"

Urg_Helper_Mock::Urg_Helper_Mock()
{
	this->urg = std::unique_ptr<FakeUrg> (new FakeUrg());
}

Urg_Helper_Mock::~Urg_Helper_Mock()
{
}

bool Urg_Helper_Mock::ConnectToUrg()
{
	std::cout << "Connecting to URG" << std::endl;
	try
	{
		std::string userHome = std::string(std::getenv("USERPROFILE"));

		std::shared_ptr<Serial_Mock> s = std::shared_ptr<Serial_Mock>(new Serial_Mock(userHome + std::string("\\Documents\\GitHub\\Rise\\Simulation\\imu11.txt")));
		_rotImu = std::shared_ptr<Common::IMU>(new Common::IMU(s));

		std::shared_ptr<Serial_Mock> posIMUSerial = std::shared_ptr<Serial_Mock>(new Serial_Mock(userHome + std::string("\\Documents\\GitHub\\Rise\\Simulation\\imu_pos11.txt")));
		_posIMU = std::shared_ptr<Common::IMU>(new Common::IMU(posIMUSerial));

		Sleep(1000); // sleep for 1s so IMU data can be processed...

		std::string lidarPath = userHome + std::string("\\Documents\\GitHub\\Rise\\Simulation\\lidar11.txt");

		if (!urg->open(lidarPath.c_str()))
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
	return Urg_Helper::GetScanFromUrg();
}
