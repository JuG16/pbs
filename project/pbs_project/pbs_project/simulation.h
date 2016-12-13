#pragma once
#include "typedef.h"
#include "sphere.h"
#include "vehicle.h"
#include "plane.h"
#include "utility.h"
#include "rigidbody.h"
#include "gjk_algorithm.h"
#include <iostream>
#include <iomanip>


//#define DEBUG

class simulation
{
public:
	simulation(std::vector<rigidbody*> objects, int_t n_objects):n_objects_(n_objects)
	{
		//initialize system -> set initial positions and velocities of all components
		
		forces_.resize(6 * n_objects_); 
		velocities_.resize(6 * n_objects_);
		const int_t s = 3 * n_objects_*n_objects_; //how large is s (correct)?
		eta_.resize(s);
		coll_resolve_.resize(s);
		jacobian_.resize(s,6 * n_objects_);

		
		fext_.resize(6 * n_objects_);
		fext_.setZero(6 * n_objects_);
		for (int i = 0; i < n_objects_; ++i)
		{
		
			//set external forces
			fext_(6 * i + 1) = -9.81*objects[i]->getmass();

			//set initial velocities
			velocities_.segment<3>(6 * i) = objects[i]->getvel();
			velocities_.segment<3>(6 * i + 3) = objects[i]->getangvel();
		}


		//initialization of massmatrix using triplets (basically dont need right?)
		massmatrix_.resize(6 * n_objects_, 6 * n_objects_);
		massmatrix_.reserve(12 * n_objects_);
		std::vector<Eigen::Triplet<real_t>> triplets;
		triplets.reserve(12 * n_objects_);
		for(int i=0;i<n_objects;++i)
		{
			triplets.push_back(Eigen::Triplet<real_t>(6 * i, 6 * i, objects[i]->getmass()));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 1, 6 * i + 1, objects[i]->getmass()));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 2, 6 * i + 2, objects[i]->getmass()));

			mat3d tempinertia = objects[i]->getinertia();
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
		
		massmatrix_.setFromTriplets(triplets.begin(), triplets.end());
		massmatrix_.makeCompressed();

		//initialization of massmatrixinv using properperties for inversion of blockdiagonal matrix (assuming nonsingularity)
		massmatrixinv_.resize(6 * n_objects_, 6 * n_objects_);
		massmatrixinv_.reserve(12 * n_objects_);
		triplets.clear();
		triplets.reserve(12 * n_objects_);
		for (int i = 0; i<n_objects_; ++i)
		{
			real_t massinv = objects[i]->getmass_inv();
			triplets.push_back(Eigen::Triplet<real_t>(6 * i, 6 * i, massinv));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 1, 6 * i + 1, massinv));
			triplets.push_back(Eigen::Triplet<real_t>(6 * i + 2, 6 * i + 2, massinv));

			//compute inverse of inertia (ok since only 3x3 matrix)
			mat3d tempinertia = objects[i]->getinertia_inv_glob();
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

		massmatrixinv_.setFromTriplets(triplets.begin(), triplets.end());
		massmatrixinv_.makeCompressed();

	}

	void step(std::vector<rigidbody*> objects, real_t dt)
	{
		//sphere is a container with random access "[]"
		//every element is of type "sphere"
		
#ifdef DEBUG

		std::cout << "step start..." << std::endl;

#endif // DEBUG
		//comput J matrix
		//needs to compute r somehow?
		//can probably leave out all noncontact spheres (not setting instead of zero[or wrong sign])
		//use "grid" to get potential neighbours
		//store all elements in a grid in a vector 
		//to delete swap with last and remove last (efficience)
		//for general shapes need to compute contact point first!! (irrelevant for spheres though)
		std::vector<Eigen::Triplet<real_t>> triplets;
		triplets.reserve(3*n_objects_*n_objects_);
		int_t row = 0;
		const real_t pen_coeff = 0.1 / dt;
		const real_t alpha = -0.5; //"amount" of bouncing
		coll_resolve_.setZero();
		for (int i = 0; i < n_objects_; ++i) //compute all interactions twice like this?
		{
#ifdef DEBUG

			std::cout << i << std::endl;

#endif // DEBUG
			for (int j = i+1; j < n_objects_; ++j)
			{
				bool contact = false;
				vec3d contactpoint;
				vec3d n;
				real_t pen_depth;

				if (objects[i]->issphere()&&objects[j]->issphere()) //use fact that sphere sphere collision is far easier (makes sense since there are a lot of spheres)
				{
#ifdef DEBUG

					std::cout << "sphere-sphere" << std::endl;

#endif // DEBUG
					//check after collision wether objects are moving towards each other (else collision has been resolved in previous timestep (but not fast enough)
					//sphere-sphere collision (need to add friction)
					if ((objects[i]->getpos() - objects[j]->getpos()).norm() < (objects[i]->getrad() + objects[j]->getrad()))
					{
						contactpoint = objects[j]->getpos()+(objects[i]->getpos() - objects[j]->getpos()) / 2.;
						n = (contactpoint - objects[i]->getpos()).normalized();
						if (n.dot(objects[i]->getvel()) > 0 || n.dot(objects[j]->getvel())<0) //need to check for both objects since 1 could be static (vel=0)
						{
							pen_depth = ((objects[i]->getrad() + objects[j]->getrad()) - (objects[i]->getpos() - objects[j]->getpos()).norm()) / 2.;
							contact = true;
						}
#ifdef DEBUG

						std::cout << contactpoint << std::endl;

#endif // DEBUG

					}
				}
				else //first check wit AABB then use gjk and eta algorithm
				{	
#ifdef DEBUG

					std::cout << "box" << std::endl;

#endif // DEBUG
					bool precond = false;
					if (objects[i]->issphere())
					{
						vec3d minpos;
						vec3d maxpos;
						objects[j]->computeAABB(minpos, maxpos);
						if (intersect(minpos, maxpos, objects[i]))
						{
							precond = true;
						}
					}
					else if (objects[j]->issphere())
					{
						vec3d minpos;
						vec3d maxpos;
						objects[i]->computeAABB(minpos, maxpos);
						if (intersect(minpos, maxpos, objects[j]))
						{
							precond = true;
						}
					}
					else
					{
						vec3d minpos1;
						vec3d maxpos1;
						vec3d minpos2;
						vec3d maxpos2;
						objects[i]->computeAABB(minpos1, maxpos1);
						objects[j]->computeAABB(minpos2, maxpos2);
						if (intersect(minpos1, maxpos1, minpos2, maxpos2))
						{
							precond = true;
						}
					}
					if (precond)
					{
						gjk_algorithm gjk = gjk_algorithm();
						if (gjk.collisiondetection(objects[i], objects[j]))
						{
#ifdef DEBUG
							std::cout << "collision detected" << std::endl;
							std::cout << objects[i]->getpos() << std::endl;
							std::cout << objects[j]->getpos() << std::endl;
#endif
							if (gjk.computecontactpoint(objects[i], objects[j], contactpoint, n, pen_depth))
							{
								if (n.dot(objects[j]->getvel()) > 0 || n.dot(objects[i]->getvel()) < 0)//need to check for both objects since one could be static (vel=0)
								{
									contact = true;
								}
							}
							else
							{
								std::cout << "unexpected error in GJK algorithm. Step terminates" << std::endl;
							}
#ifdef DEBUG

							std::cout << contactpoint << std::endl;

#endif // DEBUG
						}
					}
				}
				if (contact)
				{
					
					//using notion of paper for variables assuming sphere[i] is object 1 and sphere[j] object 2

					vec3d r1 = contactpoint - objects[i]->getpos();
					vec3d r2 = contactpoint - objects[j]->getpos();

					//use coll_resolve vector ->put -beta*C
					coll_resolve_(row) = -pen_coeff/dt*(objects[j]->getpos() + r2 - objects[i]->getpos() - r1).dot(n)+alpha*(objects[j]->getvel()+objects[j]->getangvel().cross(r2)-objects[i]->getvel()-objects[i]->getangvel().cross(r1)).dot(n);

					//normal constraints
					//need to add contactcaching
					//object 1
					n.normalize();
					const vec3d r1xn = -r1.cross(n); //- since only - crossprod gets used
					const vec3d r2xn = r2.cross(n);
					
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i, -n(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 1, -n(1)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 2, -n(2)));
					
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 3, r1xn(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 4, r1xn(1)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 5, r1xn(2)));
					//object 2
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j, n(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 1, n(1)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 2, n(2)));

					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 3, r2xn(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 4, r2xn(1)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 5, r2xn(2)));
				
					++row;
					//friciton constraints
					vec3d tempvec; //some vector != n
					if (n(0) > 0.1)
					{
						tempvec(0) = -n(0);
						tempvec(1) = n(1);
						tempvec(2) = n(2);
					}
					else if (n(1) > 0.1)
					{
						tempvec(0) = n(0);
						tempvec(1) = -n(1);
						tempvec(2) = n(2);
					}
					else
					{
						tempvec(0) = n(0);
						tempvec(1) = n(1);
						tempvec(2) = -n(2);
					}
					//2 tangential vectors to the normal (how are they related to each other? how to compute?)
					const vec3d u1 = n.cross(tempvec); //vector orthogonal to n (tangential to bodies)
					const vec3d u2 = n.cross(u1); //vector orthogonal to n and u1 

					const vec3d r1xu1 = -r1.cross(u1); //- since only - crossprod gets used
					const vec3d r2xu1 = r2.cross(u1);
					const vec3d r1xu2 = -r1.cross(u2);
					const vec3d r2xu2 = r2.cross(u2);
					//can we already sum up this values?

					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i, -u1(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 1, -u1(1)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 2, -u1(2)));

					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 3, r1xu1(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 4, r1xu1(1)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 5, r1xu1(2)));
					//object 2
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j, u1(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 1, u1(1)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 2, u1(2)));

					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 3, r2xu1(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 4, r2xu1(1)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 5, r2xu1(2)));

					++row;

					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i, -u2(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 1, -u2(1)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 2, -u2(2)));

					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 3, r1xu2(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 4, r1xu2(1)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * i + 5, r1xu2(2)));
					//object 2
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j, u2(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 1, u2(1)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 2, u2(2)));

					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 3, r2xu2(0)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 4, r2xu2(1)));
					triplets.push_back(Eigen::Triplet<real_t>(row, 6 * j + 5, r2xu2(2)));

					row++;

				}
			}
			
			//do sphere-car interaction here
			//AABB for candidates (interesection with non spheres doesnt work yet)
			/*if (intersect(minpos, maxpos, spheres[i]))
			{
				
				//compute planes for each side (of original box)
				//test for intersection
				//->probably use function for return early	
				vec3d contactpoint;
				if (computecontact(car, spheres[i], contactpoint)) //probably wrong
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
				
			}*/
#ifdef DEBUG

			std::cout << "check0" << std::endl;

#endif // DEBUG

		}
		//do car-environment interaction here (same)

		jacobian_.setFromTriplets(triplets.begin(), triplets.end()); //does this work multiple times (reinitializing)
		jacobian_.makeCompressed();
		
		//probably have to use some iterative solvers instead ?
		//compute eta
		eta_ = 1./dt*coll_resolve_-jacobian_*(1. / dt*velocities_ + massmatrixinv_*fext_);
		//can we assume J*M^-1*J^T is sparse?
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
		solver.compute(jacobian_*massmatrixinv_*jacobian_.transpose()); //problem: if all zero can get any result
		vector_t lambda = solver.solve(eta_);
		/*for (int i = 0; i < lambda.size(); ++i)
		{
			lambda(i) = std::max(lambda(i), 0.);
		}*/
#ifdef DEBUG

		std::cout << "#iterations:" << solver.iterations() << std::endl;
		std::cout << "error" << solver.error() << std::endl;

#endif // DEBUG
		
		//update velocities schwachsinn??
		//solver.analyzePattern(massmatrix_);
		//solver.factorize(massmatrix_);
		//velocities_=solver.solve(dt*(jacobian_.transpose()*lambda+fext_)+massmatrix_*velocities_)
		
		//correct?
#ifdef DEBUG

		std::cout << "lambda:" << std::endl;
		std::cout << lambda << std::endl;
		std::cout << "fext:" << std::endl;
		std::cout << fext_ << std::endl;

#endif // DEBUG
		velocities_ = massmatrixinv_*(dt*(jacobian_.transpose()*lambda + fext_)) + velocities_;
		for (int i = 0; i < n_objects_; ++i)
		{
			objects[i]->setvel(velocities_.segment<3>(6*i)); //can use blocking?
			objects[i]->setangvel(velocities_.segment<3>(6*i+3));//same
		}
#ifdef DEBUG

		std::cout << "check4" << std::endl;

#endif // DEBUG
		//set velocity of car
		for (int i = 0; i < n_objects_; ++i)
		{
			//position update
			objects[i]->setpos(objects[i]->getpos() + dt*objects[i]->getvel());
			//rotation update
			objects[i]->setquat(quataddcwise(objects[i]->getquat(),quatscalar(dt/2.,quatwmult(objects[i]->getquat(), objects[i]->getangvel())))); //terrible since all functions are self made (probably better to use vec4d)
#ifdef DEBUG

			std::cout <<  i << std::endl;
			std::cout << objects[i]->getpos() << std::endl;
			std::cout << objects[i]->getvel() << std::endl;

#endif // DEBUG
		}
	}
private:
	//probably can do a lot "matrixfree" instead of sparse matrix calculation for performance
	//ordering for matrices and vectors is: spheres, car, environment 
	int_t n_objects_;
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
	vector_t coll_resolve_;
	vector_t fext_; //denoted as F_ext in paper "Iterative Dynamics" (6nx1) declared as member to prevent memory alloc every step (not sure that this works)
	//only consists of gravity in -z axis, ->do as constant somehow (performance)
	real_t beta_; //parameter used for overlapping denoted as "beta" in paper "Iterative Dynamics" should be <1/dt for stability
};

//compute J from constraints:
