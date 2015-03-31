#ifndef IMU_H
#define IMU_H
#include "Quaternion_Common.h"
#include "Point_Common.h"
#include <queue>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>

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

	// ASSUMPTION: SLERP algorithm works for position vectors as well as quaternions.
	class IMU
	{
	public:
		IMU(std::shared_ptr<Serial> serial, bool syncRequired = true);

		// thread-safe via mutex
		void readQuaternion();
		long startCollecting();

		// User-facing queue side
		bool isHistoryEmpty();
		Quaternion findTimestamp(long timeStamp);

		void stopThread();

		~IMU();
	

	private:
		// WARNING: ABANDON ALL HOPE YE WHO ENTER HERE
		// _imuThread _MUST_ be declared last because of some
		//   esoteric C++ initializer list rule that the members are
		//   initialized in the order in which they are declared here
		//   and not in the order in which they appear in the init list.
		//   This wouldn't normally be a problem, but in this case the
		//   thread begins immediately and attempts utilizing these members
		//   before they've been initialized.  Meh.

		std::shared_ptr<Serial> _imuSerial;
		std::queue<Quaternion_Time> _positionHistory;
		std::unique_ptr<std::mutex> _queueLock;
		bool _isSendingQuatData;
		bool _syncRequired;
		std::atomic<bool> _running;
		std::atomic<bool> _waiting;
		std::thread _imuThread;
	};
}
#endif