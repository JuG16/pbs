#pragma once
#include "typedef.h"

class vehicle
{
public:
	vehicle(vec3d pos, real_t mass = 1, vec3d vel = vec3d(0, 0, 0)) :pos_(pos), mass_(mass), vel_(vel)
	{}
	inline void setpos(const vec3d pos)
	{
		pos_ = pos;
	}
	inline void setpos(const real_t x, const real_t y, const real_t z)
	{
		pos_(0) = x;
		pos_(1) = y;
		pos_(2) = z;
	}
	inline void setvel(vec3d vel)
	{
		vel_ = vel;
	}
	inline void setvel(real_t x, real_t y, real_t z)
	{
		vel_(0) = x;
		vel_(1) = y;
		vel_(2) = z;
	}
	inline vec3d getpos()const
	{
		return pos_;
	}
	inline vec3d getvel()const
	{
		return vel_;
	}
private:
	vec3d pos_;
	vec3d vel_;
	real_t mass_;
};