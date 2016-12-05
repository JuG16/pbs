#pragma once

#include "typedef.h"


class rigidbody
{
public:
	rigidbody(vec3d pos, mat3d inertia = Eigen::MatrixXd::Identity(3, 3), real_t mass = 1, vec3d vel = vec3d(0, 0, 0), vec3d angvel=vec3d(0,0,0), quaternion_t quat = quaternion_t(1, 0, 0, 0)) :pos_(pos), inertia_(inertia),  mass_(mass), vel_(vel), angvel_(angvel), quaternion_(quat)
	{}
	virtual void addtoscene(ISceneManager* smgr, IVideoDriver* driver)const = 0;

	virtual void updateScene(ISceneManager* smgr, IVideoDriver* driver)const = 0;

	virtual vec3d getfarthestpoint(vec3d const &dir)const = 0;

	virtual std::vector<vec3d> getcorners()const = 0;

	virtual bool issphere()const = 0;

	virtual real_t getrad()const = 0; //only call for spheres (verify with issphere)

	virtual real_t getlength()const = 0; //only call for objects that use this (verify with issphere)
	virtual real_t getheight()const = 0; //only call for objects that use this (verify with issphere)
	virtual real_t getwidth()const = 0; //only call for objects that use this (verify with issphere)

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
protected:
	vec3d pos_;
	mat3d inertia_; //Trägheitsmoment in bodyframe
	vec3d vel_;
	vec3d angvel_;
	quaternion_t quaternion_; //used for rotation denoted as q in paper "Iterative Dynamics" (probably irrelevant for sphere)	
	real_t mass_;
	//assume it to be a box (for collision handling)
};