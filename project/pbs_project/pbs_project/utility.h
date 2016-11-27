#pragma once
#include "typedef.h"

inline quaternion_t quatmult(quaternion_t const &q1, quaternion_t const &q2)
{
	quaternion_t qres;

	qres.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
	qres.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());

	return qres;
}