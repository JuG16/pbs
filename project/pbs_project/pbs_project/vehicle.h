#pragma once
#include "typedef.h"

class vehicle
{
public:
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
	inline void getpos(vec3d &pos)const
	{
		pos = pos_;
	}
	inline void getvel(vec3d &vel)const
	{
		vel = vel_;
	}
private:
	vec3d pos_;
	vec3d vel_;
	real_t mass_;
};