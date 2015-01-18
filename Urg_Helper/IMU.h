#ifndef IMU_H
#define IMU_H
#include "Quaternion_Common.h"
#include "Point_Common.h"
#include <queue>
#include <string>

namespace std 
{
	class thread;
	class mutex;
}

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
		IMU(std::wstring imuPath);

		// thread-safe via boost::lockfree::queue
		void readQuaternion();
		long getTimeStamp();

		// User-facing queue side
		bool isHistoryEmpty();
		Quaternion findTimestamp(long timeStamp, long tolerance = 10);

		std::thread* make_thread();

		~IMU();
	

	private:
		Serial* _imuSerial;
		std::queue<Quaternion_Time> _positionHistory;
		std::mutex *_queueLock;
		bool _isSendingQuatData;
	};
}
#endif