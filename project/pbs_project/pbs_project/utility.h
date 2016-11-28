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

//computes scalar multiplication of a quaternion
inline quaternion_t quatscalar(const real_t scalar, quaternion_t const &q)
{
	return quaternion_t(scalar*q.w(), scalar*q.x(), scalar*q.y(), scalar*q.z());
}

//elementwise compares the vectors and sets min elementwise
inline void elemwisemin(vec3d &min, vec3d const &curr)
{
	min(0) = (min(0) < curr(0)) ? min(0) : curr(0);
	min(1) = (min(1) < curr(1)) ? min(1) : curr(1);
	min(2) = (min(2) < curr(2)) ? min(2) : curr(2);
}

//elementwise compares the vectors and sets max elementwise
inline void elemwisemax(vec3d &max, vec3d const &curr)
{
	max(0) = (max(0) < curr(0)) ? max(0) : curr(0);
	max(1) = (max(1) < curr(1)) ? max(1) : curr(1);
	max(2) = (max(2) < curr(2)) ? max(2) : curr(2);
}