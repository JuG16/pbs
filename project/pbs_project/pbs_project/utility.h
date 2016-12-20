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

//cwise addition of quaternions
inline quaternion_t quataddcwise(quaternion_t const &q1, quaternion_t const &q2)
{
	return quaternion_t(q1.w() + q2.w(), q1.x() + q2.x(), q1.y() + q2.y(), q1.z() + q2.z());
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
	max(0) = (max(0) > curr(0)) ? max(0) : curr(0);
	max(1) = (max(1) > curr(1)) ? max(1) : curr(1);
	max(2) = (max(2) > curr(2)) ? max(2) : curr(2);
}

//checks wether an AABB and a sphere intersect (doesnt work for general box-sphere intersection)
inline bool intersect(vec3d const &minpos, vec3d const &maxpos, rigidbody* const s)
{
	//find closest point to sphere within box
	real_t x = std::max(minpos.x(), std::min(s->getpos().x(), maxpos.x()));
	real_t y = std::max(minpos.y(), std::min(s->getpos().y(), maxpos.y()));
	real_t z = std::max(minpos.z(), std::min(s->getpos().z(), maxpos.z()));

	vec3d pt = vec3d(x, y, z);

	//rather use this -> dont have to compute square root
	const real_t dist2 = (s->getpos() - pt).dot(s->getpos() - pt);
	return dist2 < (s->getrad()*s->getrad());

	/*real_t dist = (s->getpos() - pt).norm();

	return dist < s->getrad();*/
}


//checks wether 2 AABB intersect (doesnt work for general box-sphere intersection)
inline bool intersect(vec3d const &minpos1, vec3d const &maxpos1, vec3d const &minpos2, vec3d const &maxpos2)
{
	//find closest point to sphere within box
	return ((minpos1.x() <= maxpos2.x() && maxpos1.x() >= minpos2.x()) &&
		(minpos1.y() <= maxpos2.y() && maxpos1.y() >= minpos2.y()) &&
		(minpos1.z() <= maxpos2.z() && maxpos1.z() >= minpos2.z()));

}