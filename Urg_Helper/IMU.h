#ifndef IMU_H
#define IMU_H
#include "Quaternion_Common.h"
#include "Point_Common.h"
#include <queue>
#include <string>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

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
		IMU(std::string imuPath);

		// thread-safe via boost::lockfree::queue
		void readQuaternion();
		long getTimeStamp();

		// User-facing queue side
		bool isHistoryEmpty();
		Quaternion findTimestamp(long timeStamp, long tolerance = 10);

		boost::thread* make_thread();

		~IMU();
	

	private:
		Serial* _imuSerial;
		std::queue<Quaternion_Time> _positionHistory;
		boost::mutex _queueLock;
		bool _isSendingQuatData;
	};
}
#endif