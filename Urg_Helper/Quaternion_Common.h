#ifndef QUATERNION_COMMON_H
#define QUATERNION_COMMON_H

#include "Point_Common.h"
namespace Common
{
	class PointXYZ;

	class Quaternion
	{
	public:
		float Q0, Q1, Q2, Q3;

		Quaternion();
		Quaternion(PointXYZ p);

		PointXYZ GetPoint() const;

		double roll();
		double pitch();
		double yaw();
	};
}
#endif
