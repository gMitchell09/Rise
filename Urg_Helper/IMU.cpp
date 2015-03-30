#include "Stdafx.h"
#include "IMU.h"
#include "SerialClass.h"

#include <thread>
#include <mutex>
#include <limits>

namespace Common
{
	IMU::IMU(std::shared_ptr<Serial> serial) :
		_isSendingQuatData(false), 
		_running(true),
		_waiting(true),
		_imuSerial(serial), 
		_queueLock(std::unique_ptr<std::mutex>(new std::mutex())),
		_imuThread(std::bind(&IMU::readQuaternion, this))
	{
		if (!_imuSerial->IsConnected())
			throw "Could not connect to arduino";
	}

	IMU::~IMU()
	{
		this->stopThread();
		if (_imuThread.joinable())
			_imuThread.join();
	}

	void IMU::stopThread()
	{
		_running = false;
	}

	void IMU::readQuaternion()
	{
		while (_waiting);
		while (_running)
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

			// clean off buffer
			while (_imuSerial->Pop() != '~' && timeout >= 0) 
			{
				Sleep(3);
				timeout -= 3;
			}

			if (timeout < 0) continue;
		
			int bytesRead = _imuSerial->ReadToChar(receivedTime, 'D', 1024);
			if (bytesRead <= 0) continue;
			receivedTime[bytesRead] = '\0';
			imu_time = atol(receivedTime);
			if (imu_time == 0) continue;

			bytesRead = _imuSerial->ReadToChar(receivedTime, 'E', 1024);

			if (bytesRead < 16) continue; // discard partial reads

			q.x = ((float*)receivedTime)[0];
			q.y = ((float*)receivedTime)[1];
			q.z = ((float*)receivedTime)[2];
			q.w = ((float*)receivedTime)[3];

			Quaternion_Time qt;
			qt.q = q;
			qt.timestamp = imu_time;

			_queueLock->lock();
			_positionHistory.push(qt);
			_queueLock->unlock();
		}

		std::cout << "Finished IMU!!!" << std::endl;
	}

	long IMU::getTimeStamp()
	{
		std::cout << "Getting timestamp" << std::endl;
		if (_isSendingQuatData)
			return -1;

		char receivedTime[1024];
		long timeout = 1000;

		_imuSerial->WriteData("T", 1);
		Sleep(50);
		char c = _imuSerial->Pop();
		while (c != 'T')
		{
			if (timeout < 0) return -1;
			Sleep(10);
			timeout -= 10;
			c = _imuSerial->Pop();
			if (timeout % 100) _imuSerial->WriteData("T", 1);
		}
		int read_d = _imuSerial->ReadToChar(receivedTime, 'E', 1024);
		receivedTime[read_d] = '\0';

		_waiting = false;
		std::cout << "Timestamp: " << receivedTime << std::endl;
		return atol(receivedTime);
	}

	bool IMU::isHistoryEmpty()
	{
		return _positionHistory.empty();
	}

	Quaternion IMU::findTimestamp(long timestamp)
	{
		Quaternion_Time qt_prev;
		Quaternion_Time qt_post;

		bool isValid = false;
		
		if (_positionHistory.empty()) 
		{
			return Quaternion();
		}

		do
		{
			_queueLock->lock();
			qt_prev = _positionHistory.front();
			if (qt_prev.timestamp > timestamp) 
			{
				_queueLock->unlock();
				break;
			}
			_positionHistory.pop();
			
			if (_positionHistory.empty())
			{
				_queueLock->unlock();
				break;
			}
			qt_post = _positionHistory.front();
			_queueLock->unlock();
			
			isValid = (qt_prev.timestamp <= timestamp && qt_post.timestamp >= timestamp);
		}
		while (qt_prev.timestamp < timestamp && !isValid && !_positionHistory.empty());

		if (!isValid)
		{
			std::cout << "Empty: " << _positionHistory.size() << std::endl << "Prev: " << qt_prev.timestamp << " : " << "current: " << timestamp << " : post: " << qt_post.timestamp << std::endl;
		}

		if (isValid) 
		{
			std::cout << "Slerp!" << std::endl;
			return Quaternion(qt_prev.q, qt_post.q, float(timestamp - qt_prev.timestamp) / float(qt_post.timestamp));
		}

		else return Quaternion();
	}
}