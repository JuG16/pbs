#pragma once

#include "typedef.h"
//maybe want to add rotation at some point
class sphere
{
public:
	sphere(vec3d pos, mat3d inertia=Eigen::MatrixXd::Identity(3,3), real_t radius = 1, real_t mass = 1, vec3d vel = vec3d(0, 0, 0)) :pos_(pos), inertia_(inertia), radius_(radius), mass_(mass), vel_(vel)
	{}

	inline void setpos(const vec3d pos)
	{
		pos_ = pos;
	}
	inline void setpos(const real_t x,const real_t y,const real_t z)
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
	inline void setangvel(vec3d angvel)
	{
		angvel_ = angvel;
	}
	inline void setangvel(real_t x, real_t y, real_t z)
	{
		angvel_(0) = x;
		angvel_(1) = y;
		angvel_(2) = z;
	}
	inline vec3d getpos()const
	{
		return pos_;
	}
	inline vec3d getvel()const
	{
		return vel_;
	}
	inline vec3d getangvel()const
	{
		return angvel_;
	}
	inline real_t getrad()const
	{
		return radius_;
	}
	inline mat3d getinertia()const
	{
		return inertia_;
	}
	inline real_t getmass()const
	{
		return mass_;
	}

private:
	vec3d pos_;
	mat3d inertia_; //Trägheitsmoment
	vec3d vel_;
	vec3d angvel_;
	real_t mass_;
	real_t radius_;

};