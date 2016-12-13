#pragma once
#include <limits>
#include "typedef.h"
#include "utility.h"
#include "rigidbody.h"

class box :public rigidbody
{
public:
	box(vec3d pos, mat3d inertia = Eigen::MatrixXd::Identity(3, 3), real_t mass = 10000000, real_t length = 1, real_t width = 1, real_t height = 1, vec3d vel = vec3d(0, 0, 0), vec3d angvel = vec3d(0, 0, 0), quaternion_t quat = quaternion_t(1, 0, 0, 0)) : rigidbody(pos, inertia, mass, vel, angvel, quat)
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
		NodeBox->setMaterialTexture(0, driver->getTexture("../media/rockwall.jpg"));
		NodeBox->setPosition(vector3df(pos_.x(), pos_.y(), pos_.z()));
		NodeBox->setRotation(quattoirr(quaternion_));
	}

	vec3d getfarthestpoint(vec3d const &dir)const override
	{
		//notice that a corner will at least be one of the farthest points no matter the direction so its sufficient to consider corners
		//to get the one in the corresponding direction one simply needs to maximize the dot-product (maximum will always be >0)

		const mat3d rotmat = quaternion_.toRotationMatrix();
		real_t maxval = -std::numeric_limits<real_t>::max();;
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
		currcorner = pos_ + rotmat*vec3d(0.5*-length_, 0.5*-width_, 0.5*-height_);
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