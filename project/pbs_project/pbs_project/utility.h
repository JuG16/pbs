#pragma once
#include "typedef.h"

//computes the quaternion product
inline quaternion_t quatmult(quaternion_t const &q1, quaternion_t const &q2)
{
	quaternion_t qres;

	qres.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
	qres.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());

	return qres;
}

//computes the quaternion product of a quaternion and a vec3 extending the vector with a 0 at the beginning (as used in paper)
inline quaternion_t quatwmult(quaternion_t const &q1, vec3d const &w)
{
	quaternion_t qres;

	qres.w() = - q1.vec().dot(w);
	qres.vec() = q1.w() * w + q1.vec().cross(w);

	return qres;
}