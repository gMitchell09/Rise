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
		while (_waiting) std::this_thread::yield();
		while (_running)
		{
			long timeout = 100;
			Quaternion q;

			if (!_isSendingQuatData)
			{
				_imuSerial->WriteData("1", 1);
				_isSendingQuatData = true;
				Sleep(50);
			}

			Serial::Packet p;
			while (p.type != Serial::PacketTypes::kQuaternion && timeout > 0)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				timeout -= 10;
				p = _imuSerial->GetPacket(Serial::PacketTypes::kQuaternion);
			}

			if (p.type != Serial::PacketTypes::kQuaternion)
			{
				std::this_thread::yield();
				continue;
			}

			assert(p.length == sizeof(float));

			q.x = ((float*)p.data)[0];
			q.y = ((float*)p.data)[1];
			q.z = ((float*)p.data)[2];
			q.w = ((float*)p.data)[3];

			Quaternion_Time qt;
			qt.q = q;
			qt.timestamp = p.timestamp;

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

		long timeout = 1000;

		_imuSerial->WriteData("T", 1);
		Sleep(50);
		Serial::Packet p;
		while (p.type != Serial::PacketTypes::kTimeStamp && timeout > 0)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			timeout -= 10;
			p = _imuSerial->GetPacket(Serial::PacketTypes::kTimeStamp);
		}

		if (p.type != Serial::PacketTypes::kTimeStamp) throw "Could not get timestamp";

		_waiting = false;
		std::cout << "Timestamp: " << p.timestamp << std::endl;
		return p.timestamp;
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