#pragma once
#include "typedef.h"

class plane
{
public:
	plane(vec3d a, vec3d b, vec3d c)
	{
		normal_ = (a - b).cross(c - b);
		offset_ = -a.dot(normal_);
	}

	inline vec3d rayintersect(vec3d const &origin, vec3d const &dir)
	{
		const real_t s = -1 * (origin.dot(normal_) + offset_)/(dir.dot(normal_));
		return origin + s*dir;
	}
	
	inline vec3d getnormal()const
	{
		return normal_;
	}

private:
	//planeequation is normal_*(x,y,z)+offset_=0
	vec3d normal_;
	real_t offset_;

};