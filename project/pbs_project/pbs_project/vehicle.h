#pragma once
#include <limits>
#include "typedef.h"
#include "utility.h"
#include "sphere.h"
#include "plane.h"
#include "rigidbody.h"

class vehicle: public rigidbody
{
public:
	vehicle(vec3d pos, mat3d inertia = 10000*Eigen::MatrixXd::Identity(3, 3), real_t mass = 100, real_t length = 5, real_t width = 5, real_t height = 5, vec3d vel = vec3d(-10, -5,0), vec3d rot=vec3d(0,0,0), vec3d angvel = vec3d(0, 0, 0), quaternion_t quat = quaternion_t(1, 0, 0, 0)) : rigidbody(pos, inertia, mass, vel, rot, angvel, quat)
	{
		inertia_inv_ = inertia_.inverse();
		inertia_inv_glob_ = quaternion_.toRotationMatrix()*inertia_inv_*quaternion_.toRotationMatrix().transpose();
		mass_inv_ = 1. / mass_;
		length_ = length;
		width_ = width;
		height_ = height;
	}

	void addtoscene(ISceneManager* smgr, IVideoDriver* driver)const override
	{
		vector3df TScale = vector3df(length_, width_, height_);
		scene::ISceneNode *NodeBox = smgr->addCubeSceneNode(1.0f);
		NodeBox->setScale(TScale);
		NodeBox->setMaterialFlag(video::EMF_LIGHTING, 1);
		NodeBox->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
		//NodeBox->setMaterialTexture(0, driver->getTexture("../media/176.jpg"));
		NodeBox->setMaterialTexture(0, driver->getTexture("../media/rockwall.jpg"));
		NodeBox->setPosition(vector3df(pos_.x(), pos_.y(), pos_.z()));
		NodeBox->setRotation(vector3df(rot_.x(), rot_.y(), rot_.z()));
		//NodeBox->setRotation(quattoirr(quaternion_));
	}



	vec3d getfarthestpoint(vec3d const &dir)const override
	{
		//notice that a corner will at least be one of the farthest points no matter the direction so its sufficient to consider corners
		//to get the one in the corresponding direction one simply needs to maximize the dot-product (maximum will always be >0)

		const mat3d rotmat = quaternion_.toRotationMatrix();
		real_t maxval = -std::numeric_limits<real_t>::max();
		vec3d maxcorner;
		std::vector<vec3d> corners = this->getcorners();
		for (auto currcorner : corners)
		{
			if (currcorner.dot(dir) > maxval)
			{
				maxval = currcorner.dot(dir);
				maxcorner = currcorner;
			}
		}
		return maxcorner;
	}

	std::vector<vec3d> getcorners()const override
	{
		std::vector<vec3d> res;
		vec3d currcorner;
		const mat3d rotmat = quaternion_.toRotationMatrix();
		currcorner = pos_ + rotmat*vec3d(0.5*length_, 0.5*width_, 0.5*height_);
		res.push_back(currcorner);
		currcorner = pos_ + rotmat*vec3d(0.5*length_, 0.5*width_, 0.5*-height_);
		res.push_back(currcorner);
		currcorner = pos_ + rotmat*vec3d(0.5*length_, 0.5*-width_, 0.5*height_);
		res.push_back(currcorner);
		currcorner = pos_ + rotmat*vec3d(0.5*length_, 0.5*-width_, 0.5*-height_);
		res.push_back(currcorner);
		currcorner = pos_ + rotmat*vec3d(0.5*-length_, 0.5*width_, 0.5*height_);
		res.push_back(currcorner);
		currcorner = pos_ + rotmat*vec3d(0.5*-length_, 0.5*width_, 0.5*-height_);
		res.push_back(currcorner);
		currcorner = pos_ + rotmat*vec3d(0.5*-length_, 0.5*-width_, 0.5*height_);
		res.push_back(currcorner);
		currcorner = pos_ + rotmat*vec3d(0.5*length_, 0.5*-width_, 0.5*-height_);
		res.push_back(currcorner);
		return res;
	}

	bool issphere()const override
	{
		return false;
	}

	real_t getrad()const override
	{
		return 0;
	}

	void computeAABB(vec3d &minpos, vec3d &maxpos)const override
	{
		mat3d rotmat = quaternion_.toRotationMatrix();
		std::vector<vec3d> corners = this->getcorners();
		minpos = corners.at(0);
		maxpos = corners.at(0);
		for (auto currpos : corners)
		{
			elemwisemin(minpos, currpos);
			elemwisemax(maxpos, currpos);
		}
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