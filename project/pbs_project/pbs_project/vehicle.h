#pragma once
#include "typedef.h"
#include "utility.h"
#include "sphere.h"
#include "plane.h"

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
	mat3d inertia_; //Tr�gheitsmoment in bodyframe
	vec3d vel_;
	vec3d angvel_;
	quaternion_t quaternion_; //used for rotation denoted as q in paper "Iterative Dynamics" (probably irrelevant for sphere)	
	real_t mass_;
	//assume it to be a box (for collision handling)
	real_t length_;
	real_t width_;
	real_t height_;
};

//computes the contactpoint of a (general) box and a sphere returns true if found and false otherwise
bool computecontact(vehicle const &car, sphere const &s, vec3d &contactpoint)
{
	vec3d a;
	vec3d b;
	vec3d c;
	real_t length = car.getlength();
	real_t width = car.getwidth();
	real_t height = car.getheight();
	real_t r;
	real_t t;
	//front plane (regarding nonrotated state)
	a = car.getpos() + car.getquat().toRotationMatrix()*vec3d(length, -width, height);
	b = car.getpos() + car.getquat().toRotationMatrix()*vec3d(length, -width, -height);
	c = car.getpos() + car.getquat().toRotationMatrix()*vec3d(length, width, -height);
	plane p1 = plane(a, b, c);
	vec3d inter1 = p1.rayintersect(s.getpos(), p1.getnormal());
	//find s and t as in notes
	t = (inter1.y() - b.y() - (a.y() - b.y()) / (a.x() - b.x())*(inter1.x() - b.x())) / ((c.y() - b.y()) - (a.y() - b.y()) / (a.x() - b.x())*(c.x() - b.x()));
	r = (inter1.x() - b.x() - t*(c.x() - b.x())) / (a.x() - b.x());
	if (r >= 0 && r <= 1 && t >= 0 && t <= 1)
	{
		contactpoint = inter1;
		return true;
	}

	//right plane (regarding nonrotated state)
	a = car.getpos() + car.getquat().toRotationMatrix()*vec3d(length, width, height);
	b = car.getpos() + car.getquat().toRotationMatrix()*vec3d(length, width, -height);
	c = car.getpos() + car.getquat().toRotationMatrix()*vec3d(-length, width, -height);
	plane p2 = plane(a, b, c);
	vec3d inter2 = p2.rayintersect(s.getpos(), p2.getnormal());
	//find s and t as in notes
	t = (inter2.y() - b.y() - (a.y() - b.y()) / (a.x() - b.x())*(inter2.x() - b.x())) / ((c.y() - b.y()) - (a.y() - b.y()) / (a.x() - b.x())*(c.x() - b.x()));
	r = (inter2.x() - b.x() - t*(c.x() - b.x())) / (a.x() - b.x());
	if (r >= 0 && r <= 1 && t >= 0 && t <= 1)
	{
		contactpoint = inter2;
		return true;
	}

	//back plane (regarding nonrotated state)
	a = car.getpos() + car.getquat().toRotationMatrix()*vec3d(-length, width, height);
	b = car.getpos() + car.getquat().toRotationMatrix()*vec3d(-length, width, -height);
	c = car.getpos() + car.getquat().toRotationMatrix()*vec3d(-length,- width, -height);
	plane p3 = plane(a, b, c);
	vec3d inter3 = p3.rayintersect(s.getpos(), p3.getnormal());
	//find s and t as in notes
	t = (inter3.y() - b.y() - (a.y() - b.y()) / (a.x() - b.x())*(inter3.x() - b.x())) / ((c.y() - b.y()) - (a.y() - b.y()) / (a.x() - b.x())*(c.x() - b.x()));
	r = (inter3.x() - b.x() - t*(c.x() - b.x())) / (a.x() - b.x());
	if (r >= 0 && r <= 1 && t >= 0 && t <= 1)
	{
		contactpoint = inter3;
		return true;
	}

	//left plane (regarding nonrotated state)
	a = car.getpos() + car.getquat().toRotationMatrix()*vec3d(-length, -width, height);
	b = car.getpos() + car.getquat().toRotationMatrix()*vec3d(-length, -width, -height);
	c = car.getpos() + car.getquat().toRotationMatrix()*vec3d(length, -width, -height);
	plane p4 = plane(a, b, c);
	vec3d inter4 = p4.rayintersect(s.getpos(), p4.getnormal());
	//find s and t as in notes
	t = (inter4.y() - b.y() - (a.y() - b.y()) / (a.x() - b.x())*(inter4.x() - b.x())) / ((c.y() - b.y()) - (a.y() - b.y()) / (a.x() - b.x())*(c.x() - b.x()));
	r = (inter4.x() - b.x() - t*(c.x() - b.x())) / (a.x() - b.x());
	if (r >= 0 && r <= 1 && t >= 0 && t <= 1)
	{
		contactpoint = inter4;
		return true;
	}

	//top plane (regarding nonrotated state)
	a = car.getpos() + car.getquat().toRotationMatrix()*vec3d(-length, -width, height);
	b = car.getpos() + car.getquat().toRotationMatrix()*vec3d(length, -width, height);
	c = car.getpos() + car.getquat().toRotationMatrix()*vec3d(length, width, height);
	plane p5 = plane(a, b, c);
	vec3d inter5 = p5.rayintersect(s.getpos(), p5.getnormal());
	//find s and t as in notes
	t = (inter5.y() - b.y() - (a.y() - b.y()) / (a.x() - b.x())*(inter5.x() - b.x())) / ((c.y() - b.y()) - (a.y() - b.y()) / (a.x() - b.x())*(c.x() - b.x()));
	r = (inter5.x() - b.x() - t*(c.x() - b.x())) / (a.x() - b.x());
	if (r >= 0 && r <= 1 && t >= 0 && t <= 1)
	{
		contactpoint = inter5;
		return true;
	}

	//bottom plane (regarding nonrotated state)
	a = car.getpos() + car.getquat().toRotationMatrix()*vec3d(length, -width, -height);
	b = car.getpos() + car.getquat().toRotationMatrix()*vec3d(-length, -width, -height);
	c = car.getpos() + car.getquat().toRotationMatrix()*vec3d(-length, width, -height);
	plane p6 = plane(a, b, c);
	vec3d inter6 = p6.rayintersect(s.getpos(), p6.getnormal());
	//find s and t as in notes
	t = (inter6.y() - b.y() - (a.y() - b.y()) / (a.x() - b.x())*(inter6.x() - b.x())) / ((c.y() - b.y()) - (a.y() - b.y()) / (a.x() - b.x())*(c.x() - b.x()));
	r = (inter6.x() - b.x() - t*(c.x() - b.x())) / (a.x() - b.x());
	if (r >= 0 && r <= 1 && t >= 0 && t <= 1)
	{
		contactpoint = inter6;
		return true;
	}
	return false;
}