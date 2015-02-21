#include "Stdafx.h"
#include "Point_Common.h"

namespace Common
{
	PointXYZ::PointXYZ() {}
	PointXYZ::PointXYZ(float x, float y, float z) { this->x = x; this->y = y; this->z = z; }

	float PointXYZ::dot(const PointXYZ& other)
	{
		return x * other.x + y * other.y + z * other.z;
	}

	PointXYZ operator*(float scalar, const PointXYZ& other)
	{
		return PointXYZ(other.x * scalar, other.y * scalar, other.z * scalar);
	}

	PointXYZ operator*(const PointXYZ& other, float scalar)
	{
		return PointXYZ(other.x * scalar, other.y * scalar, other.z * scalar);
	}

	PointXYZ operator+(const PointXYZ& first, const PointXYZ& second)
	{
		return PointXYZ(first.x + second.x, first.y + second.y, first.z + second.z);
	}

	PointXYZ PointXYZ::Rotate(const Quaternion& quat)
	{
		PointXYZ u = quat.GetPoint();
		float s = quat.w;
		PointXYZ value = 2.0f * u.dot(*this) * u
			+ (s*s - u.dot(u)) * (*this)
			+ 2.0f * s * u.Cross(*this);
		return u;
	}

	PointXYZ PointXYZ::Cross(PointXYZ other)
	{
		PointXYZ result(y * other.z - z * other.y,
						z * other.x - x * other.z,
						x * other.y - y * other.x);
		return result;
	}
}