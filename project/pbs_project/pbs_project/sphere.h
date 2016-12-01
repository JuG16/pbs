#pragma once

#include "typedef.h"
#include "rigidbody.h"
//maybe want to add rotation at some point
class sphere:public rigidbody
{
public:
	sphere(vec3d pos, mat3d inertia=Eigen::MatrixXd::Identity(3,3), real_t radius = 1, real_t mass = 1, vec3d vel = vec3d(0, 0, 0), quaternion_t quat = quaternion_t(1, 0, 0, 0)) :rigidbody(pos, inertia, mass, vel, quat)
	{
		radius_ = radius;
	}

	void addtoscene(ISceneManager* smgr, IVideoDriver* driver) override
	{
		scene::ISceneNode *Node = smgr->addSphereSceneNode(radius_, 32);
		Node->setMaterialFlag(video::EMF_LIGHTING, 1);
		Node->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
		Node->setMaterialTexture(0, driver->getTexture("../media/stones.jpg"));
	}

	inline real_t getrad()const
	{
		return radius_;
	}

private:
	real_t radius_;

};