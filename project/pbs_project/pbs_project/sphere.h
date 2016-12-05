#pragma once

#include "typedef.h"
#include "rigidbody.h"
#include <iostream>
//maybe want to add rotation at some point
class sphere:public rigidbody
{
public:
	sphere(vec3d pos, mat3d inertia=Eigen::MatrixXd::Identity(3,3), real_t radius = 5, real_t mass = 1, vec3d vel = vec3d(0, 0, 0), vec3d angvel=vec3d(0,0,0), quaternion_t quat = quaternion_t(1, 0, 0, 0)) :rigidbody(pos, inertia, mass, vel, angvel, quat)
	{
		radius_ = radius;
		
	}

	void addtoscene(ISceneManager* smgr, IVideoDriver* driver)const override
	{
		std::cout << "position: " << pos_ << std::endl;
		std::cout <<"radius: "<< radius_ << std::endl;
		scene::ISceneNode *Node = smgr->addSphereSceneNode(radius_, 32);
		Node->setMaterialFlag(video::EMF_LIGHTING, 1);
		Node->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
		Node->setMaterialTexture(0, driver->getTexture("../media/stones.jpg"));
		Node->setPosition(vector3df(pos_.x(), pos_.y(), pos_.z()));
	}

	vec3d getfarthestpoint(vec3d const &dir)const override
	{
		return this->getpos()+radius_*dir.normalized();
	}

	std::vector<vec3d> getcorners()const override
	{
		std::vector<vec3d> res;
		return res;
	}

	bool issphere()const override
	{
		return true;
	}

	real_t getrad()const override
	{
		return radius_;
	}

	real_t getlength()const override
	{
		return 0;
	}
	real_t getwidth()const override
	{
		return 0;
	}
	real_t getheight()const override
	{
		return 0;
	}

private:
	real_t radius_;

};