#pragma once
#include "typedef.h"
#include "utility.h"

class vehicle
{
public:
	vehicle(vec3d pos, real_t mass = 1, real_t length=1, real_t width=1, real_t height=1, vec3d vel = vec3d(0, 0, 0), quaternion_t quat=quaternion_t(1,0,0,0)) :pos_(pos), mass_(mass), length_(length), width_(width), height_(height), vel_(vel), quaternion_(quat)
	{}
	void computeAABB(vec3d &minpos, vec3d &maxpos)
	{
		mat3d rotmat = quaternion_.toRotationMatrix();
		vec3d currpos = pos_ + rotmat*vec3d(length_, width_, height_);
		minpos = currpos;
		maxpos = currpos;
		currpos= pos_ + rotmat*vec3d(length_, width_, -height_);
		elemwisemin(minpos, currpos);
		elemwisemax(maxpos, currpos);
		currpos = pos_ + rotmat*vec3d(length_, -width_, height_);
		elemwisemin(minpos, currpos);
		elemwisemax(maxpos, currpos);
		currpos = pos_ + rotmat*vec3d(length_, -width_, -height_);
		elemwisemin(minpos, currpos);
		elemwisemax(maxpos, currpos);
		currpos = pos_ + rotmat*vec3d(-length_, width_, height_);
		elemwisemin(minpos, currpos);
		elemwisemax(maxpos, currpos);
		currpos = pos_ + rotmat*vec3d(-length_, width_, -height_);
		elemwisemin(minpos, currpos);
		elemwisemax(maxpos, currpos);
		currpos = pos_ + rotmat*vec3d(-length_, -width_, height_);
		elemwisemin(minpos, currpos);
		elemwisemax(maxpos, currpos);
		currpos = pos_ + rotmat*vec3d(-length_, -width_, -height_);
		elemwisemin(minpos, currpos);
		elemwisemax(maxpos, currpos);
	}
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
	inline void setquat(quaternion_t q)
	{
		quaternion_ = q;
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
	inline quaternion_t getquat()const
	{
		return quaternion_;
	}
	inline mat3d getinertia()const
	{
		return inertia_;
	}
	inline real_t getmass()const
	{
		return mass_;
	}
	inline real_t getlength()const
	{
		return length_;
	}
	inline real_t getwidth()const
	{
		return width_;
	}
	inline real_t getheight()const
	{
		return height_;
	}
private:
	vec3d pos_;
	mat3d inertia_; //Trägheitsmoment in bodyframe
	vec3d vel_;
	vec3d angvel_;
	quaternion_t quaternion_; //used for rotation denoted as q in paper "Iterative Dynamics" (probably irrelevant for sphere)	
	real_t mass_;
	//assume it to be a box (for collision handling)
	real_t length_;
	real_t width_;
	real_t height_;
};