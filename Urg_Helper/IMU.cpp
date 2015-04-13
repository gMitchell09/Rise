#include "Stdafx.h"
#include "IMU.h"
#include "SerialClass.h"

#include <thread>
#include <mutex>
#include <limits>

namespace Common
{
	IMU::IMU(std::shared_ptr<Serial> serial, bool syncRequired, bool noWrite) :
		_isSendingQuatData(false), 
		_running(true),
		_waiting(syncRequired),
		_syncRequired(syncRequired),
		_noWrite(noWrite),
		_imuSerial(serial), 
		_queueLock(std::unique_ptr<std::mutex>(new std::mutex())),
		_imuThread(std::bind(&IMU::readQuaternion, this))
	{
		if (!_imuSerial->IsConnected())
			throw std::exception("Could not connect to arduino");
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
				if (!_noWrite) _imuSerial->WriteData("1", 1);
				_isSendingQuatData = true;
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}

			Serial::Packet p = _imuSerial->GetPacket(Serial::PacketTypes::kQuaternion);
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

			if (p.length != 4 * sizeof(float))
			{
				assert(p.length == 4 * sizeof(float));
			}

			q.w = ((float*)p.data)[0];
			q.x = ((float*)p.data)[1];
			q.y = ((float*)p.data)[2];
			q.z = ((float*)p.data)[3];

			Quaternion_Time qt;
			qt.q = q;
			qt.timestamp = p.timestamp;

			_queueLock->lock();
			if (!_positionHistory.empty() && qt.timestamp < _positionHistory.back().timestamp)
			{
				Common::Quaternion_Time qt_tmp(_positionHistory.back());
				_positionHistory.back() = qt;
				_positionHistory.push(qt_tmp);
			}
			else
			{
				_positionHistory.push(qt);
			}
			_queueLock->unlock();
		}

		std::cout << "Finished IMU!!!" << std::endl;
	}

	long IMU::startCollecting()
	{
		std::cout << "Getting timestamp" << std::endl;
		if (_isSendingQuatData)
			return -1;

		long timeout = 1000;

		if (!_noWrite) std::cout << _imuSerial->WriteData("T", 1) << std::endl;
		Serial::Packet p;
		while (p.type != Serial::PacketTypes::kTimeStamp && timeout > 0)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			timeout -= 10;
			p = _imuSerial->GetPacket(Serial::PacketTypes::kTimeStamp);
		}

		if (p.type != Serial::PacketTypes::kTimeStamp) 
			throw std::exception("Could not get timestamp");

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
			Quaternion q(qt_prev.q, qt_post.q, float(timestamp - qt_prev.timestamp) / float(qt_post.timestamp), _syncRequired);
			
			if (!_syncRequired)
			{
				//std::cout << "Point: " << qt_prev.q.x << ", " << qt_prev.q.y << ", " << qt_prev.q.z << std::endl;
				//std::cout << "Interpolated: " << q.x << ", " << q.y << ", " << q.z << std::endl;
				return qt_prev.q;
			}

			return q;
		}

		else return Quaternion();
	}
}