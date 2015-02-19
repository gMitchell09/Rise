#ifndef IMU_H
#define IMU_H
#include "Quaternion_Common.h"
#include "Point_Common.h"
#include <queue>
#include <string>
#include <thread>
#include <mutex>

class Serial;
namespace Common
{
	class Quaternion;
	class PointXYZ;

	struct Quaternion_Time
	{
		Quaternion q;
		long timestamp;
	};

	class IMU
	{
	public:
		IMU(Serial *serial);

		// thread-safe via mutex
		void readQuaternion();
		long getTimeStamp();

		// User-facing queue side
		bool isHistoryEmpty();
		Quaternion findTimestamp(long timeStamp, long tolerance = 250);

		void stopThread();

		~IMU();
	

	private:
		Serial* _imuSerial;
		std::queue<Quaternion_Time> _positionHistory;
		std::mutex *_queueLock;
		bool _isSendingQuatData;
		bool _running;
		std::thread imuThread;

	};
}
#endif