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
		massmatrix_.resize(6 * (n_spheres+1), 6 * (n_spheres+1);
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
	}

	template <typename container>
	void step(container &spheres, vehicle &car)
	{
		//sphere is a container with random access "[]"
		//every element is of type "sphere"
		for (int i = 0; i < n_spheres; ++i)
		{
			velocities(6 * i, 6 * i + 2) = spheres[i].getvel();
			velocities(6 * i + 3, 6 * i + 5) = spheres[i].getangvel();
		}

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
		//compute forces and store them in forces vector
	}
private:
	int_t n_spheres_;
	vector_t forces_; //is denoted as F_c in paper "Iterative Dynamics"
	vector_t velocities_; //is denoted as V in paper "Iterative Dynamics"
	smatrix_t massmatrix_; //is denoted a M in paper "iterative Dynamics"
};

