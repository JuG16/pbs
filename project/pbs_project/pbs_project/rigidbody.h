#pragma once

#include "typedef.h"


class rigidbody
{
public:
	rigidbody(vec3d pos, mat3d inertia = Eigen::MatrixXd::Identity(3, 3), real_t mass = 1, vec3d vel = vec3d(0, 0, 0), vec3d angvel=vec3d(0,0,0), quaternion_t quat = quaternion_t(1, 0, 0, 0)) :pos_(pos), inertia_(inertia),  mass_(mass), vel_(vel), angvel_(angvel), quaternion_(quat)
	{
		isstatic_ = false;
	}
	virtual void addtoscene(ISceneManager* smgr, IVideoDriver* driver)const = 0;

	virtual vec3d getfarthestpoint(vec3d const &dir)const = 0;

	virtual std::vector<vec3d> getcorners()const = 0;

	virtual bool issphere()const = 0;

	virtual real_t getrad()const = 0; //only call for spheres (verify with issphere)

	virtual real_t getlength()const = 0; //only call for objects that use this (verify with issphere)
	virtual real_t getheight()const = 0; //only call for objects that use this (verify with issphere)
	virtual real_t getwidth()const = 0; //only call for objects that use this (verify with issphere)
	virtual void computeAABB(vec3d &minpos, vec3d &maxpos)const = 0; //dont call for spheres (does not support this)

	inline void setstatic()
	{
		mass_inv_ = 0;
		inertia_inv_ = Eigen::MatrixXd::Zero(3,3);
		inertia_inv_glob_ = Eigen::MatrixXd::Zero(3,3);
		vel_ = Eigen::VectorXd::Zero(3);
		angvel_ = Eigen::VectorXd::Zero(3);
		isstatic_ = true;
	}
	inline void setdynamic()
	{
		mass_inv_ = 1. / mass_;
		inertia_inv_ = inertia_.inverse();
		inertia_inv_glob_ = quaternion_.toRotationMatrix()*inertia_inv_*quaternion_.toRotationMatrix().transpose();
		isstatic_ = false;
	}
	inline void setpos(const vec3d pos)
	{
		if (!isstatic_)
		{
			pos_ = pos;
		}
	}
	inline void setpos(const real_t x, const real_t y, const real_t z)
	{
		if (!isstatic_)
		{
			pos_(0) = x;
			pos_(1) = y;
			pos_(2) = z;
		}
	}
	inline void setvel(vec3d vel)
	{
		if (!isstatic_)
		{
			vel_ = vel;
		}
	}
	inline void setvel(real_t x, real_t y, real_t z)
	{
		if (!isstatic_)
		{
			vel_(0) = x;
			vel_(1) = y;
			vel_(2) = z;
		}
	}
	inline void setangvel(vec3d angvel)
	{
		if (!isstatic_)
		{
			angvel_ = angvel;
		}
	}
	inline void setangvel(real_t x, real_t y, real_t z)
	{
		if (!isstatic_)
		{
			angvel_(0) = x;
			angvel_(1) = y;
			angvel_(2) = z;
		}
	}
	inline void setquat(quaternion_t q)
	{
		if (!isstatic_)
		{
			quaternion_ = q;
			inertia_inv_glob_ = quaternion_.toRotationMatrix()*inertia_inv_*quaternion_.toRotationMatrix().transpose();
		}
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
	inline mat3d getinertia_inv_glob()const
	{
		return inertia_inv_glob_;
	}
	inline real_t getmass()const
	{
		return mass_;
	}
	inline real_t getmass_inv()const
	{
		return mass_inv_;
	}
protected:
	vec3d pos_;
	mat3d inertia_;
	mat3d inertia_inv_;
	mat3d inertia_inv_glob_;//Trägheitsmoment in bodyframe
	vec3d vel_;
	vec3d angvel_;
	quaternion_t quaternion_; //used for rotation denoted as q in paper "Iterative Dynamics" (probably irrelevant for sphere)	
	real_t mass_;
	real_t mass_inv_;
	//assume it to be a box (for collision handling)
	bool isstatic_;
};

class renderdata
{
public:
	renderdata(rigidbody* const object, vec3d const &pos, quaternion_t const &quat) :object_(object), pos_(pos), quat_(quat) {};

	inline void draw(ISceneManager* smgr, IVideoDriver* driver)const
	{
		object_->setpos(pos_);
		object_->setquat(quat_);
		object_->addtoscene(smgr, driver);
	}
private:
	rigidbody* object_;
	vec3d pos_;
	quaternion_t quat_;
};