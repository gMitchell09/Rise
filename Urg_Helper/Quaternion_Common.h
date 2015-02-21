#ifndef QUATERNION_COMMON_H
#define QUATERNION_COMMON_H

#include "Point_Common.h"
namespace Common
{
	class PointXYZ;

	class Quaternion
	{
	public:
		float x, y, z, w;

		Quaternion();
		Quaternion(PointXYZ p);
		Quaternion(float ax, float ay, float az, float aw);
		Quaternion(const Quaternion q1, const Quaternion q2, double t = 0.5);

		static Quaternion Slerp(Quaternion q1, Quaternion q2, double t);
		static Quaternion Lerp(Quaternion q1, Quaternion q2, double t);

		static float dot(const Quaternion q1, const Quaternion q2);

		PointXYZ GetPoint() const;

		double roll();
		double pitch();
		double yaw();

		float length_sqr()
		{
			return x * x + y * y + z * z + w * w;
		}
		float length()
		{
			return sqrt(length_sqr());
		}

		Quaternion normalize()
		{
			float len = length();
			if (len > 0.f && (len - 1.0f) > FLT_EPSILON)
			{
				len = 1.0f / len;
				x *= len; y *= len; z *= len; w *= len;
			}
			return *this;
		}

		Quaternion operator -() const {
			Quaternion q(-x, -y, -z, -w);
			return q;
		}

		friend Quaternion operator*(const Quaternion &q, float f)
		{
			return Quaternion (f * q.x, f * q.y, f * q.z, f * q.w);
		}
	
		friend Quaternion operator*(float f, const Quaternion &q)
		{
			return Quaternion (f * q.x, f * q.y, f * q.z, f * q.w);
		}

		friend Quaternion operator/(const Quaternion &q, float f)
		{
			return Quaternion (q.x / f, q.y / f, q.z / f, q.w / f);
		}
	
		friend Quaternion operator+(const Quaternion &a, const Quaternion &b)
		{
			return Quaternion(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
		}

		friend Quaternion operator-(const Quaternion &a, const Quaternion &b)
		{
			return Quaternion(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
		}
	};
}
#endif
