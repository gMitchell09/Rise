// Written by:		David Brown
// Final Update:	April 30th 2014
// Description:		This program combines two libraries. It connects to the LIDAR and the Arduino through a serial connection. 
//					It then gets the data from both and attempts to combine them into a 3D point cloud which is stored in a point cloud library point cloud
//

#include "stdafx.h"
#include "Urg_Helper.h"
#include "Quaternion_Common.h"
#include "Point_Common.h"

#define LIDAR_IMU_TIME_THRESHOLD 100
#define ABS(x) ((x < 0) ? (-x) : (x))

//Contructor that intiates both a the pointcloud and the drvier for the lidar
Urg_Helper::Urg_Helper() : _imuThread(NULL)
{
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
		_imu = new Common::IMU("\\\\.\\COM19");
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
		Sleep(20);
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

	_updateMutex.lock();
	_updateCloud = true;
	_updateMutex.unlock();
}

void Urg_Helper::spawnIMUThread()
{
	_imuThread = _imu->make_thread();
}

//List<long> ^ Urg_Helper::GetDataFromTheUrg(double numberofScans)
//{
//	std::vector<long> data;						//recieves the angle data directly from LiDar
//	std::vector<std::string> anglesReceived;    //Stores the step measurement from the arduino

//	List<long> ^copy = gcnew List<long>();		// This is returned to the Form and can be used for debugging
//	
//	char receivedAngle[3000];					// Character array receieved from the Arduino				
//	char receivedTime[1024];

//	double phi_Angle[200];						// Array that stores the actual angle calculated for each step
//	long time_stamp = 0;

//	_serial->WriteData("T", 1);
//	Sleep(50);
//	while (_serial->Pop() != 'T');
//	int read_d = _serial->ReadData(receivedTime, 100);
//	receivedTime[read_d] = '\0';
//	urg->set_sensor_time_stamp(atol(receivedTime));

//	//Begin measurement for both Lidar and then Arduino
//	urg->start_measurement(qrk::Urg_driver::Distance, numberofScans, 0);
//	_serial->WriteData("1", 1);
//	// Sleeps for 10 seconds to wait for URG and especially arduino to finish its measurements
//	Sleep(2000);
//	//Gets step data from the arudino
//	//_serial->ReadData(receivedAngle, 3000);

//	pcl::PointXYZ *tempPoint;					// Point passed to the point cloud for temporary storage
//	//This loops for the number of scans to collect each piece of data
//	for(int j =0; j < numberofScans; j++)
//	{
//		Common::Quaternion q;
//		bool validIMU = false;
//		long imu_time = 0;
//		if (!urg->get_distance(data, &time_stamp)) {		// Gets the data from the URG
//			return copy;									// if failes doesnt return any data
//		}

//		while (ABS(time_stamp - imu_time) > LIDAR_IMU_TIME_THRESHOLD 
//			&& imu_time < time_stamp + LIDAR_IMU_TIME_THRESHOLD)
//		{
//			while (_serial->Pop() != '~') Sleep(3);
//			int bytesRead = _serial->ReadToChar(receivedTime, 'D', 1024);
//			if (bytesRead <= 0) continue;
//			receivedTime[bytesRead] = '\0';
//			imu_time = atol(receivedTime);
//			if (imu_time == 0) continue;
//			bytesRead = _serial->ReadToChar(receivedTime, 'E', 1024);
//			
//			if (bytesRead < 16)
//			{
//				continue;
//			}
//			q.Q0 = ((float*)receivedTime)[0];
//			q.Q1 = ((float*)receivedTime)[1];
//			q.Q2 = ((float*)receivedTime)[2];
//			q.Q3 = ((float*)receivedTime)[3];

//			validIMU = true;
//		}

//		if (!validIMU)
//		{
//			std::cout << "Skipping data" << std::endl;
//			continue;
//		}

//		for (int i = 0; i < data.size(); i++)
//		{
//			q.Q1 = 0;
//			q.Q3 = 0;
//			double mag = sqrt(q.Q0 * q.Q0 + q.Q2 * q.Q2);
//			q.Q0 /= mag;
//			q.Q2 /= mag;
//			double angle = 2 * acos(q.Q2);
//			/*angle = time_stamp/1000.0 * 360;*/
//			tempPoint = CreatePoint(i, data[i], angle, false);
//			cloud->push_back(*tempPoint);
//			delete tempPoint;
//		}
//	}
//	
//	return copy;
//}

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
	pcl::visualization::CloudViewer cloudviewer("Cloud");
	cloudviewer.showCloud(pcl::PointCloud <pcl::PointXYZ>::Ptr(cloud));
	// Visualization stops after a short time, this loop keeps it going.
	while (!cloudviewer.wasStopped ())
	{
	}
	return true;
}
