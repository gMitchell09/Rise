#include "Stdafx.h"
#include "Quaternion_Common.h"

#include <limits>

namespace Common
{
	Quaternion::Quaternion()
	{
		x = y = z = w = std::numeric_limits<float>::signaling_NaN();
	}

	Quaternion::Quaternion(PointXYZ p)
	{
		x = p.x;
		y= p.y;
		z = p.z;
		w = 0;
	}

	Quaternion::Quaternion(float aw, float ax, float ay, float az)
		: x(ax), y(ay), z(az), w(aw) {}

	Quaternion::Quaternion(const Quaternion q1, const Quaternion q2, double interval)
	{
		Quaternion tmp = Quaternion::Slerp(q1, q2, interval);
		x = tmp.x; y = tmp.y; z = tmp.z; w = tmp.w;
	}

	float Quaternion::dot(const Quaternion q1, const Quaternion q2)
	{
		return q1.x*q2.x + q1.y*q2.y + q1.z*q2.z + q1.w*q2.w;
	}

	Quaternion Quaternion::Lerp(Quaternion q1, Quaternion q2, double t)
	{
		return ((q1 + t * (q2 - q1)).normalize());
	}

	Quaternion Quaternion::Slerp(Quaternion q1, Quaternion q2, double t)
	{
		Quaternion q3;
		float dot = Quaternion::dot(q1, q2);

		if (dot < 0)
		{
			dot = -dot;
			q3 = -q2;
		}
		else q3 = q2;

		if (dot < 0.95f)
		{
			float angle = acosf(dot);
			return (q1 * sinf(angle*(1-t)) + q3*sinf(angle*t))/sinf(angle);
		}
		else
		{
			return Lerp(q1, q3, t);
		}
	}

	PointXYZ Quaternion::GetPoint() const
	{
		return PointXYZ(x, y, z);
	}

	double Quaternion::yaw()
	{
		y = 0;
		w = 0;
		double mag = sqrt(x * x + z * z);
		x /= mag;
		z /= mag;
		double angle = 2 * acos(z);

		return angle;
	}

	double Quaternion::pitch()
	{
		return atan2(2 * x * w - 2 * y * z, 1 - 2 * x * x - 2 * z * z);
	}

	double Quaternion::roll()
	{
		return atan2(2 * y * w - 2 * x * z, 1 - 2 * y * y - 2 * z * z);
	}
}