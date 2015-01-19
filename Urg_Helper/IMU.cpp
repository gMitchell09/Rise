#include "Stdafx.h"
#include "IMU.h"
#include "SerialClass.h"

#include <thread>
#include <mutex>

namespace Common
{
	IMU::IMU(std::string imuPath) : _isSendingQuatData(false)
	{
		_queueLock = new std::mutex();
		_imuSerial = new Serial(imuPath.c_str());
		if (!_imuSerial->IsConnected())
			throw "Could not connect to arduino";
	}

	void IMU::readQuaternion()
	{
		long timeout = 100;
		char receivedTime[1024];
		long imu_time;
		Quaternion q;

		if (!_isSendingQuatData)
		{
			_imuSerial->WriteData("1", 1);
			_isSendingQuatData = true;
		}

		while (_imuSerial->Pop() != '~') 
		{
			if (timeout < 0) return;
			Sleep(3);
			timeout -= 3;
		}

		
		int bytesRead = _imuSerial->ReadToChar(receivedTime, 'D', 1024);
		if (bytesRead <= 0) return;
		receivedTime[bytesRead] = '\0';
		imu_time = atol(receivedTime);
		if (imu_time == 0) return;

		if (bytesRead < 16) return; // discard partial reads

		q.Q0 = ((float*)receivedTime)[0];
		q.Q1 = ((float*)receivedTime)[1];
		q.Q2 = ((float*)receivedTime)[2];
		q.Q3 = ((float*)receivedTime)[3];

		Quaternion_Time qt;
		qt.q = q;
		qt.timestamp = imu_time;

		_queueLock->lock();
		_positionHistory.push(qt);
		_queueLock->unlock();
	}

	long IMU::getTimeStamp()
	{
		if (_isSendingQuatData)
			return -1;

		char receivedTime[1024];
		long timeout = 1000;

		_imuSerial->WriteData("T", 1);
		Sleep(50);
		while (_imuSerial->Pop() != 'T')
		{
			if (timeout < 0) return -1;
			Sleep(10);
			timeout -= 10;
		}
		int read_d = _imuSerial->ReadData(receivedTime, 1024);
		receivedTime[read_d] = '\0';

		return atol(receivedTime);
	}

	bool IMU::isHistoryEmpty()
	{
		return _positionHistory.empty();
	}

	Quaternion IMU::findTimestamp(long timestamp, long tolerance)
	{
		Quaternion_Time qt;
		bool isValid = false;

		do
		{
			_queueLock->lock();
			qt = _positionHistory.front();
			_positionHistory.pop();
			_queueLock->unlock();

			isValid = (abs(qt.timestamp - timestamp) > tolerance);
		}
		while (qt.timestamp < timestamp + tolerance &&
			abs(qt.timestamp - timestamp) > tolerance);

		if (isValid) return qt.q;
		else return Quaternion();
	}

	std::thread* IMU::make_thread()
	{
		return new std::thread(std::bind(&IMU::readQuaternion, this));
	}

	IMU::~IMU()
	{
		delete _imuSerial;
	}
}