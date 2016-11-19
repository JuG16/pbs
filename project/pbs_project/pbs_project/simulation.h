#pragma once
#include "typedef.h"
#include "sphere.h"
#include "vehicle.h"

template <typename container>
void init(container &spheres, vehicle &car)
{
	//initialize system -> set initial positions and velocities of all components
	//sphere is a container with random access "[]"
	//every element is of type "sphere"
}

template <typename container>
void step(container &spheres, const int_t n_spheres, vehicle &car)
{
	//sphere is a container with random access "[]"
	//every element is of type "sphere"
	vector_t forces;
	forces.resize(3 * n_spheres);
	
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