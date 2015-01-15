#ifndef IMU_H
#define IMU_H
#include "Quaternion_Common.h"
#include "Point_Common.h"
#include <boost/lockfree/queue.hpp>
#include <string>

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
	

	private:
		Serial* _imuSerial;
		boost::lockfree::queue<Quaternion_Time> _positionHistory;
		bool _isSendingQuatData;
	};
}
#endif