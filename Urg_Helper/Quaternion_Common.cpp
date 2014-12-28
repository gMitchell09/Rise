#include "Stdafx.h"
#include "Quaternion_Common.h"

namespace Common
{
	Quaternion::Quaternion(){}
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
}