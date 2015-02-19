#include "Stdafx.h"
#include "Quaternion_Common.h"

namespace Common
{
	Quaternion::Quaternion()
	{
		Q0 = Q1 = Q2 = Q3 = -1;
	}

	Quaternion::Quaternion(PointXYZ p)
	{
		Q0 = p.x;
		Q1 = p.y;
		Q2 = p.z;
		Q3 = 0;
	}

	PointXYZ Quaternion::GetPoint() const
	{
		return PointXYZ(Q0, Q1, Q2);
	}

	double Quaternion::yaw()
	{
		Q1 = 0;
		Q3 = 0;
		double mag = sqrt(Q0 * Q0 + Q2 * Q2);
		Q0 /= mag;
		Q2 /= mag;
		double angle = 2 * acos(Q2);

		return angle;
	}

	double Quaternion::pitch()
	{
		return atan2(2 * Q0 * Q3 - 2 * Q1 * Q2, 1 - 2 * Q0 * Q0 - 2 * Q2 * Q2);
	}

	double Quaternion::roll()
	{
		return atan2(2 * Q1 * Q3 - 2 * Q0 * Q2, 1 - 2 * Q1 * Q1 - 2 * Q2 * Q2);
	}
}