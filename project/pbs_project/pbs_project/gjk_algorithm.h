#pragma once
#include <limits>

#include "typedef.h"
#include "rigidbody.h"


struct supportpoint {
	supportpoint() {}
	supportpoint(vec3d v, vec3d sup_a, vec3d sup_b) :v_(v), sup_a_(sup_a), sup_b_(sup_b)
	{}

	vec3d v_; // the minkowski difference point

			  // the individual support points			  
	vec3d sup_a_;
	vec3d sup_b_; // not actually necessary but makes things easier

	bool operator==(supportpoint const &r) const { return v_ == r.v_; }
};

struct Triangle {
	supportpoint points[3];
	vec3d n_;

	Triangle(const supportpoint &a, const supportpoint &b, const supportpoint &c) {
		points[0] = a;
		points[1] = b;
		points[2] = c;
		n_ = ((b.v_ - a.v_).cross(c.v_ - a.v_)).normalized();
	}
};

struct Edge {
	supportpoint points[2];

	Edge(const supportpoint &a, const supportpoint &b) {
		points[0] = a;
		points[1] = b;
	}
};



class gjk_algorithm
{
public:
	gjk_algorithm()
	{
		a_ = b_ = c_ = d_ = supportpoint(Eigen::VectorXd::Zero(3), Eigen::VectorXd::Zero(3), Eigen::VectorXd::Zero(3));
	}

	bool collisiondetection(rigidbody* const box1,rigidbody* const box2)
	{
		vec3d dir = vec3d(1, 1, 1);

		b_ = support(box1, box2, dir);

		dir = -c_.v_;//negative direction

		nrPointsSimplex_ = 1;
		for(int steps=0;steps<50;++steps)
		{

			a_ = support(box1, box2, dir);
			nrPointsSimplex_++;

			if (a_.v_.dot(dir) < 0)
			{
				return false;
			}

			if (containsorigin(dir)) //contains origin is "dosimplex"
			{
				return true;
			}

		}

		return false;
	}

	bool computecontactpoint(rigidbody* const box1, rigidbody* const box2, vec3d &contactpt, vec3d &colnormal, real_t &pen_depth)
	{
		const real_t growth_threshold = 0.001;
		const int_t max_iter = 30;

		triangles_.emplace_back(a_, b_, c_);
		triangles_.emplace_back(a_, c_, d_);
		triangles_.emplace_back(a_, d_, b_);
		triangles_.emplace_back(b_, d_, c_);

		for (int_t curr_iter = 0; curr_iter < max_iter; ++curr_iter)
		{
			std::vector<Triangle>::iterator min_tria_it;
			real_t min_dist = std::numeric_limits<real_t>::max();
			for (auto it = triangles_.begin(); it != triangles_.end(); ++it)
			{
				const real_t curr_dist=it->n_.dot(it->points[0].v_); //compute distance to origin in minowski difference space

				if (curr_dist < min_dist)
				{
					min_dist = curr_dist;
					min_tria_it = it;
				}
			}

			const supportpoint new_support = support(box1, box2, min_tria_it->n_);
			const real_t new_dist = min_tria_it->n_.dot(new_support.v_);
			if (new_dist - min_dist < growth_threshold)
			{
				//generate contact information (depth and point)
				vec3d bary_coords;
				barycentric_coords(min_tria_it->n_*min_dist, min_tria_it->points[0].v_, min_tria_it->points[1].v_, min_tria_it->points[2].v_, bary_coords);
				contactpt = ((bary_coords(0)*(min_tria_it->points[0].sup_a_)) + (bary_coords(1)*(min_tria_it->points[1].sup_a_)) + (bary_coords(2)*(min_tria_it->points[2].sup_a_)));
				colnormal = -min_tria_it->n_;
				pen_depth = min_dist;
				return true;
			}

			for (auto it = triangles_.begin(); it != triangles_.end();)
			{
				if (it->n_.dot(new_support.v_ - it->points[0].v_) > 0)
				{
					addedge(it->points[0], it->points[1]);
					addedge(it->points[1], it->points[2]);
					addedge(it->points[2], it->points[0]);
					continue;
				}
				++it;
			}

			for (auto edge : edges_)
			{
				triangles_.emplace_back(new_support, edge.points[0], edge.points[1]);
			}

			edges_.clear();
		}

		return false;
	}
private:

	supportpoint support(rigidbody* const a, rigidbody* const b, const vec3d dir) const
	{
		vec3d p1 = a->getfarthestpoint(dir);
		vec3d p2 = b->getfarthestpoint(-dir);

		vec3d p3 = p1 - p2;

		return supportpoint(p3, p1, p2);
	}

	void addedge(supportpoint const &a, supportpoint const &b)
	{
		for (auto it = edges_.begin(); it != edges_.end(); ++it)
		{
			if (it->points[0] == b&&it->points[1] == a)
			{
				edges_.erase(it);
				return;
			}
		}
		edges_.emplace_back(a, b);
	}

	void barycentric_coords(vec3d const &p, vec3d const &a, vec3d const &b, vec3d const &c, vec3d &u)
	{
		const vec3d v0 = b - a;
		const vec3d v1 = c - a;
		const vec3d v2 = p - a;
		const real_t d00 = v0.dot(v0);
		const real_t d01 = v0.dot(v1);
		const real_t d11 = v1.dot(v1);
		const real_t d20 = v2.dot(v0);
		const real_t d21 = v2.dot(v1);
		const real_t denom = d00*d11 - d01*d01; //correct?
		u(1) = (d11*d20 - d01*d21) / denom;
		u(2) = (d00*d21 - d01*d20) / denom;
		u(0) = 1. - u(1) - u(2);
		return;
	}
	bool containsorigin(vec3d& dir)
	{
		if (nrPointsSimplex_ == 2)
		{
			line(dir);
			return false;
		}
		else if (nrPointsSimplex_ == 3)
		{
			triangle(dir);
			return false;
		}
		else if(nrPointsSimplex_==4)
		{
			return tetrahedron(dir);
		}
		return false;
	}
	void line(vec3d& dir)
	{
		const vec3d ab = b_.v_ - a_.v_;
		dir = (ab.cross(-a_.v_)).cross(ab);
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
		const vec3d ab = b_.v_ - a_.v_;
		const vec3d ac = c_.v_ - a_.v_;
		const vec3d abc = ab.cross(ac); //normal

		if ((abc.cross(ac)).dot(-a_.v_) > 0)
		{
			dir = (ab.cross(-a_.v_)).cross(ab);
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
			if (ab.cross(abc).dot(-a_.v_) > 0)
			{
				dir = (ab.cross(-a_.v_)).cross(ab);
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
				if (abc.dot(-a_.v_) > 0)
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
		const vec3d ab = b_.v_ - a_.v_;
		const vec3d ac = c_.v_ - a_.v_;
		const vec3d ad = d_.v_ - a_.v_;

		//origin in front of triangle abc
		if ((ab.cross(ac).dot(-a_.v_)) > 0)
		{
			nrPointsSimplex_ = 3;
			triangle(dir);
			return false;
		}

		//origin in front of triangle acd
		else if (((ac.cross(ad)).dot(-a_.v_)) > 0)
		{
			nrPointsSimplex_ = 3;
			b_ = c_;
			c_ = d_;
			triangle(dir);
			return false;
		}

		//origin in fron of triangle abd
		else if (((ad.cross(ab)).dot(-a_.v_)>0))
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
	supportpoint a_, b_, c_, d_;
	std::vector<Edge>edges_;
	std::vector<Triangle>triangles_;
	int nrPointsSimplex_ = 0;
};



