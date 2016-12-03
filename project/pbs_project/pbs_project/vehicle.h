#pragma once
#include "typedef.h"
#include "utility.h"
#include "sphere.h"
#include "plane.h"
#include "rigidbody.h"

class vehicle: public rigidbody
{
public:
	vehicle(vec3d pos, mat3d inertia = Eigen::MatrixXd::Identity(3, 3), real_t mass = 1, real_t length = 1, real_t width = 1, real_t height = 1, vec3d vel = vec3d(0, 0, 0), quaternion_t quat = quaternion_t(1, 0, 0, 0)) : rigidbody(pos, inertia, mass, vel, quat)
	{
		length_ = length;
		width_ = width;
		height_ = height;
	}

	void addtoscene(ISceneManager* smgr, IVideoDriver* driver) override
	{
		vector3df TScale = vector3df(length_, width_, height_);
		scene::ISceneNode *NodeBox = smgr->addCubeSceneNode(1.0f);
		NodeBox->setScale(TScale);
		NodeBox->setMaterialFlag(video::EMF_LIGHTING, 1);
		NodeBox->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
		NodeBox->setMaterialTexture(0, driver->getTexture("../media/wall.jpg"));
	}

	vec3d getfarthestpoint(vec3d const &dir)override
	{
		//no creative idea how to do this
		return vec3d(0, 0, 0);
	}

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