#pragma once
#include "typedef.h"
#include "utility.h"
#include "rigidbody.h"

class box :public rigidbody
{
public:
	box(vec3d pos, mat3d inertia = Eigen::MatrixXd::Identity(3, 3), real_t mass = 1, real_t length = 1, real_t width = 1, real_t height = 1, vec3d vel = vec3d(0, 0, 0), quaternion_t quat = quaternion_t(1, 0, 0, 0)) : rigidbody(pos, inertia, mass, vel, quat)
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
		currpos = pos_ + rotmat*vec3d(length_, width_, -height_);
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