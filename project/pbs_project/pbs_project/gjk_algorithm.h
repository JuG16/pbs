#pragma once

#include "typedef.h"
#include "rigidbody.h"


class gjk_algorithm
{
public:
	gjk_algorithm()
	{
		a_ = b_ = c_ = d_ = Eigen::VectorXd::Zero(3);
	}

	bool CollisionDetection(const rigidbody& box1, const rigidbody& box2)
	{
		vec3d dir = vec3d(1, 1, 1);

		c_ = support(box1, box2, dir);

		dir = -c_;//negative direction

		b_ = support(box1, box2, dir);

		if (b_.dot(dir) < 0)
		{
			return false;
		}
		dir = (c_ - b_).cross(-b_);

		nrPointsSimplex_ = 2; //begin with 2 points in simplex

		int steps = 0;//avoid infinite loop
		while (steps<50)
		{
			a_ = support(box1, box2, dir);
			if (a_.dot(dir) < 0)
			{
				return false;
			}
			else
			{

				if (ContainsOrigin(dir))
				{
					return true;
				}
			}
			steps++;

		}

		return false;
	}

private:

	vec3d  support(const rigidbody& a, const rigidbody& b, const vec3d dir) const
	{
		vec3d p1 = a.getfarthestpoint(dir);
		vec3d p2 = b.getfarthestpoint(-dir);

		vec3d p3 = p1 - p2;

		return  p3;
	}

	bool ContainsOrigin(vec3d& dir)
	{
		if (nrPointsSimplex_ == 2)
		{
			return triangle(dir);
		}
		else if (nrPointsSimplex_ == 3)
		{
			return tetrahedron(dir);
		}

		return false;
	}
	bool line(vec3d& dir)
	{
		vec3d ab = b_ - a_;

		dir = ab.cross(-a_);

		c_ = b_;
		b_ = a_;
		nrPointsSimplex_ = 2;

		return false;
	}
	bool triangle(vec3d& dir) //always returns false (correct?)
	{
		vec3d ab = b_ - a_;
		vec3d ac = c_ - a_;
		vec3d abc = ab.cross(ac);


		vec3d ab_abc =ab.cross(abc);

		if (ab_abc.dot(-a_) > 0)
		{
			
			c_ = b_;
			b_ = a_;

			//dir is not ab_abc because it's not point towards the origin
			dir = ab.cross(-a_);

			//direction change; can't build tetrahedron
			return false;
		}


		vec3d abc_ac = abc.cross(ac);

		// is the origin away from ac edge? or it is in abc?
		//if a0 is in that direction than
		if (abc_ac.dot(-a_) > 0)
		{
			//keep c the same
			b_ = a_;

			//dir is not abc_ac because it's not point towards the origin
			dir = ac.cross(-a_);

			//direction change; can't build tetrahedron
			return false;
		}

		//now can build tetrahedron; check if it's above or below
		if (abc.dot(-a_) > 0)
		{
			//base of tetrahedron
			d_ = c_;
			c_ = b_;
			b_ = a_;

			//new direction
			dir = abc;
		}
		else
		{
			//upside down tetrahedron
			d_ = b_;
			b_ = a_;
			dir = -abc;
		}

		nrPointsSimplex_ = 3;

		return false;
	}
	bool tetrahedron(vec3d& dir)
	{
		vec3d ab = b_ - a_;
		vec3d ac = c_ - a_;

		//build abc triangle
		vec3d abc = ab.cross(ac);

		//CASE 1
		if (abc.dot(-a_) > 0)
		{
			//in front of triangle ABC
			//we don't have to change the ao,ab,ac,abc meanings
			checkTetrahedron(-a_, ab, ac, abc, dir);
		}


		//CASE 2:

		vec3d ad = d_ - a_;

		//build acd triangle
		vec3d acd = ac.cross(ad);

		//same direaction with ao
		if (acd.dot(-a_) > 0)
		{

			//in front of triangle ACD
			b_ = c_;
			c_ = d_;
			ab = ac;
			ac = ad;
			abc = acd;

			checkTetrahedron(-a_, ab, ac, abc, dir);
		}

		//build adb triangle
		vec3d adb = ad.cross(ab);

		//case 3:

		//same direaction with ao
		if (adb.dot(-a_) > 0)
		{

			//in front of triangle ADB

			c_ = b_;
			b_ = d_;

			ac = ab;
			ab = ad;

			abc = adb;
			checkTetrahedron(-a_, ab, ac, abc, dir);
		}


		//origin in tetrahedron
		return true;
	}

	bool checkTetrahedron(const vec3d& ao, const vec3d& ab, const vec3d& ac, const vec3d& abc, vec3d& dir)
	{

		//almost the same like triangle checks
		vec3d ab_abc = ab.cross(abc);

		if (ab_abc.dot(ao) > 0)
		{
			c_ = b_;
			b_ = a_;

			//dir is not ab_abc because it's not point towards the origin;
			//ABxA0xAB direction we are looking for
			dir = ab.cross(ao);

			//build new triangle
			// d will be lost
			nrPointsSimplex_ = 2;

			return false;
		}

		vec3d acp = abc.cross(ac);

		if (acp.dot(ao) > 0)
		{
			b_ = a_;

			//dir is not abc_ac because it's not point towards the origin;
			//ACxA0xAC direction we are looking for
			dir = ac.cross(ao);

			//build new triangle
			// d will be lost
			nrPointsSimplex_ = 2;

			return false;
		}

		//build new tetrahedron with new base
		d_ = c_;
		c_ = b_;
		b_ = a_;

		dir = abc;

		nrPointsSimplex_ = 3;

		return false;
	}

	vec3d a_, b_, c_, d_;
	int nrPointsSimplex_ = 0;
};


