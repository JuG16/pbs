#pragma once
#include "typedef.h"
#include "sphere.h"
#include "vehicle.h"


class simulation
{
public:
	template <typename container>
	simulation(container &spheres, int_t n_spheres, vehicle &car):n_spheres_(n_spheres)
	{
		//initialize system -> set initial positions and velocities of all components
		//sphere is a container with random access "[]"
		//every element is of type "sphere"
		forces_.resize(6 * (n_spheres+1)); //+1 in size is for car
		velocities_.resize(6 * (n_spheres+1));
		int_t s = 1; //how large is s???
		eta_.resize(s);
		jacobian_.resize(6 * (n_spheres + 1), s);

		//initialization of massmatrix using triplets
		massmatrix_.resize(6 * (n_spheres+1), 6 * (n_spheres+1));
		massmatrix_.reserve(12 * (n_spheres + 1));
		std::vector<Eigen::Triplet<real_t>> triplets;
		triplets.reserve(12 * (n_spheres + 1));
		for(int i=0;i<n_spheres;++i)
		{
			triplets.push_back(Eigen::Triplet(6 * i, 6 * i, sphere[i].getmass()));
			triplets.push_back(Eigen::Triplet(6 * i + 1, 6 * i + 1, sphere[i].getmass()));
			triplets.push_back(Eigen::Triplet(6 * i + 2, 6 * i + 2, sphere[i].getmass()));

			mat3d tempinertia = spheres[i].getinertia();
			triplets.push_back(Eigen::Triplet(6 * i + 3, 6 * i + 3, tempinertia(0, 0)));
			triplets.push_back(Eigen::Triplet(6 * i + 3, 6 * i + 4, tempinertia(0, 1)));
			triplets.push_back(Eigen::Triplet(6 * i + 3, 6 * i + 5, tempinertia(0, 2)));
			triplets.push_back(Eigen::Triplet(6 * i + 4, 6 * i + 3, tempinertia(1, 0)));
			triplets.push_back(Eigen::Triplet(6 * i + 4, 6 * i + 4, tempinertia(1, 1)));
			triplets.push_back(Eigen::Triplet(6 * i + 4, 6 * i + 5, tempinertia(1, 2)));
			triplets.push_back(Eigen::Triplet(6 * i + 5, 6 * i + 3, tempinertia(2, 0)));
			triplets.push_back(Eigen::Triplet(6 * i + 5, 6 * i + 4, tempinertia(2, 1)));
			triplets.push_back(Eigen::Triplet(6 * i + 5, 6 * i + 5, tempinertia(2, 2)));
		}
		triplets.push_back(Eigen::Triplet(6 * i, 6 * i, car.getmass()));
		triplets.push_back(Eigen::Triplet(6 * i + 1, 6 * i + 1, car.getmass()));
		triplets.push_back(Eigen::Triplet(6 * i + 2, 6 * i + 2, car.getmass()));

		mat3d tempinertia = car.getinertia();
		triplets.push_back(Eigen::Triplet(6 * i + 3, 6 * i + 3, tempinertia(0, 0)));
		triplets.push_back(Eigen::Triplet(6 * i + 3, 6 * i + 4, tempinertia(0, 1)));
		triplets.push_back(Eigen::Triplet(6 * i + 3, 6 * i + 5, tempinertia(0, 2)));
		triplets.push_back(Eigen::Triplet(6 * i + 4, 6 * i + 3, tempinertia(1, 0)));
		triplets.push_back(Eigen::Triplet(6 * i + 4, 6 * i + 4, tempinertia(1, 1)));
		triplets.push_back(Eigen::Triplet(6 * i + 4, 6 * i + 5, tempinertia(1, 2)));
		triplets.push_back(Eigen::Triplet(6 * i + 5, 6 * i + 3, tempinertia(2, 0)));
		triplets.push_back(Eigen::Triplet(6 * i + 5, 6 * i + 4, tempinertia(2, 1)));
		triplets.push_back(Eigen::Triplet(6 * i + 5, 6 * i + 5, tempinertia(2, 2)));

		massmatrix_.setFromTriplets(triplets.begin(), triplets.end());
		massmatrix_.makeCompressed();

		//initialization of massmatrixinv using properperties for inversion of blockdiagonal matrix (assuming nonsingularity)
		massmatrixinv_.resize(6 * (n_spheres + 1), 6 * (n_spheres + 1));
		massmatrixinv_.reserve(12 * (n_spheres + 1));
		std::vector<Eigen::Triplet<real_t>> triplets;
		triplets.reserve(12 * (n_spheres + 1));
		for (int i = 0; i<n_spheres; ++i)
		{
			real_t massinv = 1. / sphere[i].getmass();
			triplets.push_back(Eigen::Triplet(6 * i, 6 * i, massinv));
			triplets.push_back(Eigen::Triplet(6 * i + 1, 6 * i + 1, massinv));
			triplets.push_back(Eigen::Triplet(6 * i + 2, 6 * i + 2, massinv));

			//compute inverse of inertia (ok since only 3x3 matrix)
			mat3d tempinertia = spheres[i].getinertia().inverse();
			triplets.push_back(Eigen::Triplet(6 * i + 3, 6 * i + 3, tempinertia(0, 0)));
			triplets.push_back(Eigen::Triplet(6 * i + 3, 6 * i + 4, tempinertia(0, 1)));
			triplets.push_back(Eigen::Triplet(6 * i + 3, 6 * i + 5, tempinertia(0, 2)));
			triplets.push_back(Eigen::Triplet(6 * i + 4, 6 * i + 3, tempinertia(1, 0)));
			triplets.push_back(Eigen::Triplet(6 * i + 4, 6 * i + 4, tempinertia(1, 1)));
			triplets.push_back(Eigen::Triplet(6 * i + 4, 6 * i + 5, tempinertia(1, 2)));
			triplets.push_back(Eigen::Triplet(6 * i + 5, 6 * i + 3, tempinertia(2, 0)));
			triplets.push_back(Eigen::Triplet(6 * i + 5, 6 * i + 4, tempinertia(2, 1)));
			triplets.push_back(Eigen::Triplet(6 * i + 5, 6 * i + 5, tempinertia(2, 2)));
		}
		real_t massinc = 1. / car.getmass();
		triplets.push_back(Eigen::Triplet(6 * i, 6 * i, massinv));
		triplets.push_back(Eigen::Triplet(6 * i + 1, 6 * i + 1, massinv));
		triplets.push_back(Eigen::Triplet(6 * i + 2, 6 * i + 2, massinv));

		//compute inverse of inertia (ok since only 3x3 matrix)
		mat3d tempinertia = car.getinertia().inverse();
		triplets.push_back(Eigen::Triplet(6 * i + 3, 6 * i + 3, tempinertia(0, 0)));
		triplets.push_back(Eigen::Triplet(6 * i + 3, 6 * i + 4, tempinertia(0, 1)));
		triplets.push_back(Eigen::Triplet(6 * i + 3, 6 * i + 5, tempinertia(0, 2)));
		triplets.push_back(Eigen::Triplet(6 * i + 4, 6 * i + 3, tempinertia(1, 0)));
		triplets.push_back(Eigen::Triplet(6 * i + 4, 6 * i + 4, tempinertia(1, 1)));
		triplets.push_back(Eigen::Triplet(6 * i + 4, 6 * i + 5, tempinertia(1, 2)));
		triplets.push_back(Eigen::Triplet(6 * i + 5, 6 * i + 3, tempinertia(2, 0)));
		triplets.push_back(Eigen::Triplet(6 * i + 5, 6 * i + 4, tempinertia(2, 1)));
		triplets.push_back(Eigen::Triplet(6 * i + 5, 6 * i + 5, tempinertia(2, 2)));

		massmatrixinv_.setFromTriplets(triplets.begin(), triplets.end());
		massmatrixinv_.makeCompressed();

	}

	template <typename container>
	void step(container &spheres, vehicle &car, real_t dt)
	{
		//sphere is a container with random access "[]"
		//every element is of type "sphere"
		for (int i = 0; i < n_spheres; ++i)
		{
			velocities(6 * i, 6 * i + 2) = spheres[i].getvel();
			velocities(6 * i + 3, 6 * i + 5) = spheres[i].getangvel();
		}


		//comput J matrix
		//needs to compute r somehow?

		//compute contact points (really stupid for now, use better datastructure later)
		for (int i = 0; i < n_spheres; ++i)
		{
			for (int j = 0; j < n_spheres; ++j)
			{
				if ((spheres[i].getpos() - spheres[j].getpos()).norm() < spheres[i].getrad() + spheres[j].getrad())
				{
					//contact
				}
			}
		}
		//probably have to use some iterative solvers instead ?
		//compute eta
		eta = -jacobian_*(1. / dt*velocities_ + massmatrixinv_*fext_);
		//can we assume J*M*J^-1 is sparse?

		//compute lambda
		ssolver solver;
		solver.analyzePattern(jacobian_*massmatrixinv_*jacobian_.transpose());
		solver.factorize(jacobian_*massmatrixinv_*jacobian_.transpose());
		auto lambda = solver.solve(eta_);
		
		//update velocities schwachsinn??
		//solver.analyzePattern(massmatrix_);
		//solver.factorize(massmatrix_);
		//velocities_=solver.solve(dt*(jacobian_.transpose()*lambda+fext_)+massmatrix_*velocities_)
		
		//correct?
		velocities_ = massmatrixinv_*(dt*(jacobian_.transpose()*lambda + fext_)) + velocities_;

	}
private:
	//probably can do a lot "matrixfree" instead of sparse matrix calculation for performance
	int_t n_spheres_;
	vector_t forces_; //is denoted as F_c in paper "Iterative Dynamics" (6nx1) where n is the number of objects
	vector_t velocities_; //is denoted as V in paper "Iterative Dynamics" (6nx1) where n is the number of objects
	smatrix_t massmatrix_; //is denoted a M in paper "iterative Dynamics" (6nx6n) where n is the number of objects
	//massmatrix is Blockdiagonal: each object contributes wit 2 3x3 matrices, where the first is diag(m), where m is the mass of the object
	//the second is I, where I is the moment of inertia of the object
	//since only inversematrix is used for computation can just set inverse=0 for static objects (infinite mass and inertia)
	smatrix_t massmatrixinv_; //M^-1 can be precomputed since its constant
	smatrix_t jacobian_; //denoted J in paper "Iterative Dynamics" (6nxs) declared as member to prevent memory alloc every step (not sure that this works)
	vector_t eta_; //denoted as "eta" in paper "Iterative Dynamics" (sx1) declared as member to prevent memory alloc every step 
	//s =??? something with number of constraints
	smatrix_t fext_; //denoted as F_ext in paper "Iterative Dynamics" (6nx1) declared as member to prevent memory alloc every step (not sure that this works)
	//only consists of gravity in -z axis, ->do as constant somehow (performance)
};

//compute J from constraints:
