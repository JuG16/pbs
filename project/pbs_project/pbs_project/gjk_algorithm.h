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

		b_ = support(box1, box2, dir);

		dir = -c_;//negative direction

		nrPointsSimplex_ = 1;
		for(int steps=0;steps<50;++steps)
		{

			a_ = support(box1, box2, dir);
			nrPointsSimplex_++;

			if (a_.dot(dir) < 0)
			{
				return false;
			}

			if (ContainsOrigin(dir)) //contains origin is "dosimplex"
			{
				return true;
			}

		}

		return false;
	}

private:

	vec3d  support(rigidbody const &a, rigidbody const &b, const vec3d dir) const
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
			line(dir);
			return false;
		}
		else if (nrPointsSimplex_ == 3)
		{
			triangle(dir);
			return;
		}
		else if(nrPointsSimplex_==4)
		{
			return tetrahedron(dir);
		}
		return false;
	}
	void line(vec3d& dir)
	{
		vec3d ab = b_ - a_;
		dir = (ab.cross(-a_)).cross(ab);
		c_ = b_;
		b_ = a_;
		nrPointsSimplex_ = 2;
		return;

		/*
		this case cancels out because otherwise the main algorithm would already have returned false (cant get to the origin)
		else
		{
			dir = -a_;
			b_ = a_;
			nrPointsSimplex_ = 1;
			return;
		}*/

	}
	void triangle(vec3d& dir)
	{
		vec3d ab = b_ - a_;
		vec3d ac = c_ - a_;
		vec3d abc = ab.cross(ac); //normal

		if ((abc.cross(ac)).dot(-a_) > 0)
		{
			dir = (ab.cross(-a_)).cross(ab);
			c_ = b_;
			b_ = a_;
			nrPointsSimplex_ = 2;
			return;

			//other parts cancel out because of similar reason to line argument
			/*if (ac.dot(-a_) > 0)
			{
				dir = (ac.cross(-a_)).cross(ac);
				b_ = a_;
				nrPointsSimplex_ = 2;
				return;
			}
			else
			{
				if (ab.dot(-a_))
				{
					dir = (ab.cross(-a_)).cross(ab);
					c_ = b_;
					b_ = a_;
					nrPointsSimplex_ = 2;
					return;
				}
				else
				{
					dir = -a_;
					b_ = a_;
					nrPointsSimplex_ = 1;
					return;
				}
			}*/
		}
		else
		{
			if (ab.cross(abc).dot(-a_) > 0)
			{
				dir = (ab.cross(-a_)).cross(ab);
				c_ = b_;
				b_ = a_;
				nrPointsSimplex_ = 2;
				return;
				
				//same argument as above
				/*if (ab.dot(-a_))
				{
					dir = (ab.cross(-a_)).cross(ab);
					c_ = b_;
					b_ = a_;
					nrPointsSimplex_ = 2;
					return;
				}
				else
				{
					dir = -a_;
					b_ = a_;
					nrPointsSimplex_ = 1;
					return;
				}*/
			}
			else
			{
				if (abc.dot(-a_) > 0)
				{
					dir = abc;
					d_ = c_;
					c_ = b_;
					b_ = a_;
					nrPointsSimplex_ = 3;
					return;
				}
				else
				{
					dir = -abc;
					d_ = b_;
					b_ = a_;
					nrPointsSimplex_ = 3;
					return;
				}
			}
		}
	}
	bool tetrahedron(vec3d& dir)
	{
		const vec3d ab = b_ - a_;
		const vec3d ac = c_ - a_;
		const vec3d ad = d_ - a_;

		//origin in front of triangle abc
		if ((ab.cross(ac).dot(-a_)) > 0)
		{
			nrPointsSimplex_ = 3;
			triangle(dir);
			return false;
		}

		//origin in front of triangle acd
		else if (((ac.cross(ad).dot(-a_) > 0)
		{
			nrPointsSimplex_ = 3;
			b_ = c_;
			c_ = d_;
			triangle(dir);
			return false;
		}

		//origin in fron of triangle abd
		else if (((ad.cross(ab).dot(-a_))))
		{
			nrPointsSimplex_ = 3;
			c_ = d_;
			triangle(dir);
			return false;
		}

		else
		{
			return true;
		}
	}
	vec3d a_, b_, c_, d_;
	int nrPointsSimplex_ = 0;
};


