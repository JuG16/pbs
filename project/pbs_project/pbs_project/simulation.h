#pragma once
#include "typedef.h"
#include "sphere.h"
#include "vehicle.h"
#include "plane.h"
#include "utility.h"
#include <iostream>
#include <iomanip>


class simulation
{
public:
	template <typename container>
	simulation(container &spheres, int_t n_spheres, vehicle &car):n_spheres_(n_spheres)
	{
		//initialize system -> set initial positions and velocities of all components
		//sphere is a container with random access "[]"
		//every element is of type "sphere"
		forces_.resize(6 * (n_spheres+2)); //+1 in size is for car, +1 for environment
		velocities_.resize(6 * (n_spheres+2));
		const int_t s = 10; //how large is s???
		eta_.resize(s);
		jacobian_.resize(s,6 * (n_spheres + 2));

		//set external forces
		fext_.resize(6 * (n_spheres + 2));
		fext_.setZero(6 * (n_spheres + 2));
		for (int i = 0; i < n_spheres + 2; ++i)
		{
			fext_(6 * i + 2) = -9.81;
		}
		

		//set initial velocities
		for (int i = 0; i < n_spheres; ++i)
		{
			velocities_.segment<3>(6 * i) = spheres[i].getvel();
			velocities_.segment<3>(6 * i + 3) = spheres[i].getangvel();
		}

		//initialization of massmatrix using triplets
		massmatrix_.resize(6 * (n_spheres+2), 6 * (n_spheres+2));
		massmatrix_.reserve(12 * (n_spheres + 1));
		std::vector<Eigen::Triplet<real_t>> triplets;
		triplets.reserve(12 * (n_spheres + 1));
		for(int i=0;i<n_spheres;++i)
		{
			triplets.push_back(Eigen::Triplet<real_t>(6 * i, 6 * i, spheres[i].getmass()));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 1, 6 * i + 1, spheres[i].getmass()));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 2, 6 * i + 2, spheres[i].getmass()));

			mat3d tempinertia = spheres[i].getinertia();
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 3, 6 * i + 3, tempinertia(0, 0)));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 3, 6 * i + 4, tempinertia(0, 1)));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 3, 6 * i + 5, tempinertia(0, 2)));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 4, 6 * i + 3, tempinertia(1, 0)));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 4, 6 * i + 4, tempinertia(1, 1)));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 4, 6 * i + 5, tempinertia(1, 2)));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 5, 6 * i + 3, tempinertia(2, 0)));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 5, 6 * i + 4, tempinertia(2, 1)));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 5, 6 * i + 5, tempinertia(2, 2)));
		}
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres, 6 * n_spheres, car.getmass()));
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 1, 6 * n_spheres + 1, car.getmass()));
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 2, 6 * n_spheres + 2, car.getmass()));

		mat3d tempinertia = car.getinertia();
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 3, 6 * n_spheres + 3, tempinertia(0, 0)));
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 3, 6 * n_spheres + 4, tempinertia(0, 1)));
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 3, 6 * n_spheres + 5, tempinertia(0, 2)));
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 4, 6 * n_spheres + 3, tempinertia(1, 0)));
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 4, 6 * n_spheres + 4, tempinertia(1, 1)));
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 4, 6 * n_spheres + 5, tempinertia(1, 2)));
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 5, 6 * n_spheres + 3, tempinertia(2, 0)));
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 5, 6 * n_spheres + 4, tempinertia(2, 1)));
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 5, 6 * n_spheres + 5, tempinertia(2, 2)));

		massmatrix_.setFromTriplets(triplets.begin(), triplets.end());
		massmatrix_.makeCompressed();

		//initialization of massmatrixinv using properperties for inversion of blockdiagonal matrix (assuming nonsingularity)
		massmatrixinv_.resize(6 * (n_spheres + 2), 6 * (n_spheres + 2));
		massmatrixinv_.reserve(12 * (n_spheres + 1));
		triplets.clear();
		triplets.reserve(12 * (n_spheres + 1));
		for (int i = 0; i<n_spheres; ++i)
		{
			real_t massinv = 1. / spheres[i].getmass();
			triplets.push_back(Eigen::Triplet<real_t>(6 * i, 6 * i, massinv));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 1, 6 * i + 1, massinv));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 2, 6 * i + 2, massinv));

			//compute inverse of inertia (ok since only 3x3 matrix)
			mat3d tempinertia = spheres[i].getinertia().inverse();
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 3, 6 * i + 3, tempinertia(0, 0)));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 3, 6 * i + 4, tempinertia(0, 1)));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 3, 6 * i + 5, tempinertia(0, 2)));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 4, 6 * i + 3, tempinertia(1, 0)));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 4, 6 * i + 4, tempinertia(1, 1)));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 4, 6 * i + 5, tempinertia(1, 2)));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 5, 6 * i + 3, tempinertia(2, 0)));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 5, 6 * i + 4, tempinertia(2, 1)));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 5, 6 * i + 5, tempinertia(2, 2)));
		}
		real_t massinv = 1. / car.getmass();
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres, 6 * n_spheres, massinv));
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 1, 6 * n_spheres + 1, massinv));
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 2, 6 * n_spheres + 2, massinv));

		//compute inverse of inertia (ok since only 3x3 matrix)
		tempinertia = car.getinertia().inverse();
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 3, 6 * n_spheres + 3, tempinertia(0, 0)));
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 3, 6 * n_spheres + 4, tempinertia(0, 1)));
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 3, 6 * n_spheres + 5, tempinertia(0, 2)));
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 4, 6 * n_spheres + 3, tempinertia(1, 0)));
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 4, 6 * n_spheres + 4, tempinertia(1, 1)));
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 4, 6 * n_spheres + 5, tempinertia(1, 2)));
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 5, 6 * n_spheres + 3, tempinertia(2, 0)));
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 5, 6 * n_spheres + 4, tempinertia(2, 1)));
		triplets.push_back(Eigen::Triplet<real_t>(6 * n_spheres + 5, 6 * n_spheres + 5, tempinertia(2, 2)));


		//for environment just dont set value (->remains 0) and therefore has infinite mass
		massmatrixinv_.setFromTriplets(triplets.begin(), triplets.end());
		massmatrixinv_.makeCompressed();

	}

	template <typename container>
	void step(container &spheres, vehicle &car, real_t dt)
	{
		//sphere is a container with random access "[]"
		//every element is of type "sphere"
		

		std::cout << "step start" << std::endl;
		//comput J matrix
		//needs to compute r somehow?
		//can probably leave out all noncontact spheres (not setting instead of zero[or wrong sign])
		//use "grid" to get potential neighbours
		//store all elements in a grid in a vector 
		//to delete swap with last and remove last (efficience)
		//for general shapes need to compute contact point first!! (irrelevant for spheres though)
		std::vector<Eigen::Triplet<real_t>> triplets;
		triplets.reserve(1 /* how many nnz to expect?*/);
		int_t row = 0;

		//AABB for candidates for car
		vec3d minpos;
		vec3d maxpos;
		car.computeAABB(minpos, maxpos);
		for (int i = 0; i < n_spheres_; ++i) //compute all interactions twice like this?
		{
			for (int j = i+1; j < n_spheres_; ++j)
			{
				if ((spheres[i].getpos() - spheres[j].getpos()).norm() < (spheres[i].getrad() + spheres[j].getrad()))
				{
					//sphere-sphere collision (need to add friction)
					vec3d contactpoint = (spheres[i].getpos() + spheres[j].getpos()) / 2.;
					//using notion of paper for variables assuming sphere[i] is object 1 and sphere[j] object 2
					//dont think using only 1 normal works for general shapes (as presented in paper)
					vec3d n = (contactpoint - spheres[i].getpos()).normalized();
					vec3d r1 = contactpoint - spheres[i].getpos();
					vec3d r2 = contactpoint - spheres[j].getpos();
					//need to scale all values with beta_*penetration depth?
					//need to add contactcaching
					//object 1
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i, -n(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 1, -n(1)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 2, -n(2)));
					const vec3d r1xn = -r1.cross(n); //- since only - crossprod gets used
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 3, r1xn(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 4, r1xn(1)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 5, r1xn(2)));
					//object 2
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j, n(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 1, n(1)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 2, n(2)));
					const vec3d r2xn = r2.cross(n);
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 4, r2xn(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 5, r2xn(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 6, r2xn(0)));
					row++;
				}
			}
			
			//do sphere-car interaction here
			//AABB for candidates (interesection with non spheres doesnt work yet)
			if (intersect(minpos, maxpos, spheres[i]))
			{
				
				//compute planes for each side (of original box)
				//test for intersection
				//->probably use function for return early	
				vec3d contactpoint;
				if (computecontact(car, spheres[i], contactpoint))
				{
					
					vec3d n = (contactpoint - spheres[i].getpos()).normalized();
					vec3d r1 = contactpoint - spheres[i].getpos();
					vec3d r2 = contactpoint - car.getpos();
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i, -n(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 1, -n(1)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 2, -n(2)));
					const vec3d r1xn = -r1.cross(n); //- since only - crossprod gets used
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 3, r1xn(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 4, r1xn(1)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 5, r1xn(2)));
					//car
					const int_t j = (n_spheres_ + 1); //to get block of car (first after spheres)
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j, n(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 1, n(1)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 2, n(2)));
					const vec3d r2xn = r2.cross(n);
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 3, r2xn(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 4, r2xn(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 5, r2xn(0)));
					row++;
				}
				
			}
			std::cout << "check0" << std::endl;

			/*//do sphere-environment interaction here (needs some way to detect contact->function for environment)
			//contactpoint = ;
			n = (contactpoint - spheres[i].getpos()).normalized();
			r1 = contactpoint - spheres[i].getpos();
			r2 = contactpoint - car.getpos();
			triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i, -n(0)));
			triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 1, -n(1)));
			triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 2, -n(2)));
			const vec3d r1xn2 = -r1.cross(n); //- since only - crossprod gets used
			triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 3, r1xn2(0)));
			triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 4, r1xn2(1)));
			triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 5, r1xn2(2)));
			//environment (probably can remove this since has infinite mass anyways
			const int_t j = 6 * (n_spheres_ + 2); //to get block of environment (second after spheres)
			triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j, n(0)));
			triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 1, n(1)));
			triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 2, n(2)));
			const vec3d r2xn2 = r2.cross(n);
			triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 4, r2xn2(0)));
			triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 5, r2xn2(0)));
			triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 6, r2xn2(0)));*/
		}
		//do car-environment interaction here (same)

		jacobian_.setFromTriplets(triplets.begin(), triplets.end()); //does this work multiple times (reinitializing)
		jacobian_.makeCompressed();
		std::cout << "check1" << std::endl;
		
		//probably have to use some iterative solvers instead ?
		//compute eta
		eta_ = -jacobian_*(1. / dt*velocities_ + massmatrixinv_*fext_);
		//can we assume J*M^-1*J^T is sparse?
		std::cout << "check2" << std::endl;
		//compute lambda
		//do 3 steps to utilize sparsity (doesnt work since parts are not squared)
		ssolver solver;
		//solve for M^-1 J^T lambda
		/*solver.analyzePattern(jacobian_);
		solver.factorize(jacobian_);
		auto MJlambda = solver.solve(eta_);
		//solve for J^T lambda
		solver.analyzePattern(massmatrixinv_);
		solver.factorize(massmatrixinv_);
		auto Jlambda = solver.solve(MJlambda);
		//solve for lambda
		solver.analyzePattern(jacobian_.transpose());
		solver.factorize(jacobian_.transpose());
		auto lambda = solver.solve(Jlambda);*/

		//solver.analyzePattern(jacobian_*massmatrixinv_*jacobian_.transpose());
		//solver.factorize(jacobian_*massmatrixinv_*jacobian_.transpose());
		solver.compute(jacobian_*massmatrixinv_*jacobian_.transpose());
		auto lambda = solver.solve(eta_);
		std::cout << "#iterations:" << solver.iterations() << std::endl;
		std::cout << "error" << solver.error() << std::endl;
		
		//update velocities schwachsinn??
		//solver.analyzePattern(massmatrix_);
		//solver.factorize(massmatrix_);
		//velocities_=solver.solve(dt*(jacobian_.transpose()*lambda+fext_)+massmatrix_*velocities_)
		
		//correct?
		std::cout << "lambda:" << std::endl;
		std::cout << lambda << std::endl;
		std::cout << "fext:" << std::endl;
		std::cout << fext_ << std::endl;
		velocities_ = massmatrixinv_*(dt*(jacobian_.transpose()*lambda + fext_)) + velocities_;

		std::cout << "check3" << std::endl;
		for (int i = 0; i < n_spheres_; ++i)
		{
			spheres[i].setvel(velocities_.segment<3>(6*i)); //can use blocking?
			spheres[i].setangvel(velocities_.segment<3>(6*i+3));//same
		}

		std::cout << "check4" << std::endl;
		//set velocity of car
		for (int i = 0; i < n_spheres_; ++i)
		{
			//position update
			spheres[i].setpos(spheres[i].getpos() + dt*spheres[i].getvel());
			//rotation update
			spheres[i].setquat(quataddcwise(spheres[i].getquat(),quatscalar(dt/2.,quatwmult(spheres[i].getquat(), spheres[i].getangvel())))); //terrible since all functions are self made (probably better to use vec4d)
			std::cout <<  i << std::endl;
			std::cout << spheres[i].getpos() << std::endl;
			std::cout << spheres[i].getvel() << std::endl;
		}
	}
private:
	//probably can do a lot "matrixfree" instead of sparse matrix calculation for performance
	//ordering for matrices and vectors is: spheres, car, environment 
	int_t n_spheres_;
	vector_t forces_; //is denoted as F_c in paper "Iterative Dynamics" (6nx1) where n is the number of objects
	vector_t velocities_; //is denoted as V in paper "Iterative Dynamics" (6nx1) where n is the number of objects
	smatrix_t massmatrix_; //is denoted a M in paper "iterative Dynamics" (6nx6n) where n is the number of objects
	//massmatrix is Blockdiagonal: each object contributes wit 2 3x3 matrices, where the first is diag(m), where m is the mass of the object
	//the second is I, where I is the moment of inertia of the object
	//since only inversematrix is used for computation can just set inverse=0 for static objects (infinite mass and inertia) why save massmatrix?
	smatrix_t massmatrixinv_; //M^-1 can be precomputed since its constant (probably not because of global I=R*I_b*R^T) can use Eigen::Matrix3d R = q.toRotationMatrix() to get rotation matrix
	smatrix_t jacobian_; //denoted J in paper "Iterative Dynamics" (sx6n) declared as member to prevent memory alloc every step (not sure that this works)
	vector_t eta_; //denoted as "eta" in paper "Iterative Dynamics" (sx1) declared as member to prevent memory alloc every step 
	//s =??? something with number of constraints
	vector_t fext_; //denoted as F_ext in paper "Iterative Dynamics" (6nx1) declared as member to prevent memory alloc every step (not sure that this works)
	//only consists of gravity in -z axis, ->do as constant somehow (performance)
	real_t beta_; //parameter used for overlapping denoted as "beta" in paper "Iterative Dynamics" should be <1/dt for stability
};

//compute J from constraints:
