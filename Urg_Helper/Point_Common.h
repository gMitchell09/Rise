#ifndef POINT_COMMON_H
#define POINT_COMMON_H
#include "Quaternion_Common.h"

namespace Common
{
class Quaternion;
class PointXYZ
{
public:
	float x;
	float y;
	float z;

	PointXYZ();
	PointXYZ(float x, float y, float z);

	float dot(const PointXYZ& other);

	friend PointXYZ operator*(float scalar, const PointXYZ& other);
	friend PointXYZ operator*(const PointXYZ& other, float scalar);
	friend PointXYZ operator+(const PointXYZ& first, const PointXYZ& second);
	PointXYZ Rotate(const Quaternion& quat);
	PointXYZ Cross(PointXYZ other);
};
}
#endif