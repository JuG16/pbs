#include "typedef.h"
#include "simulation.h"
#include "rendering.h"
#include "sphere.h"
#include "vehicle.h"
#include "plane.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <cstdlib>

int main(int argc, char** argv)
{
	//vectors for options and corresponding descriptions for -help
	const std::vector<std::string> options({ "n_sphere","mass_sphere","radius_sphere","mass_car" });
	const std::vector<std::string> description ({ "Sets the number of spheres in the box", "sets the mass of every single sphere", "sets the radius of every single sphere", "sets the mass of the car" });


	//standard settings
	int_t n_sphere = 10;
	real_t mass_sphere = 1;
	real_t radius_sphere = 0.1;
	real_t mass_car = 100;
	if (argc == 2&&!std::strcmp(argv[1],"-help")) //strcmp gives 0 if strings are equal!!!
	{
		std::cout << "Usage: -\"option1\" <value1> -\"option2\" <value2> ..." << std::endl;
		std::cout <<std::left<< std::setw(15)<<"Options" <<std::setw(30)<< "Description"<< std::endl;
		for (int i = 0; i < options.size(); ++i)
		{
			std::cout << std::left<< std::setw(15)<< options.at(i) <<std::setw(30)<< description.at(i) << std::endl;
		}
		return 0;
	}
	if (argc % 2 == 0)
	{
		std::cout << "Usage: -\"option\" <value>" << std::endl;
		std::cout << "Type -help for available options" << std::endl;
		return 0;
	}
	for (int i = 1; i < argc; i+=2)
	{
		if (!std::strcmp(argv[i], "-n_sphere"))
		{
			n_sphere = std::atoi(argv[i + 1]);
		}
		else if (!std::strcmp(argv[i],"-mass_sphere"))
		{
			mass_sphere = std::atof(argv[i + 1]);
		}
		else if (!std::strcmp(argv[i], "-radius_sphere"))
		{
			radius_sphere = std::atof(argv[i + 1]);
		}
		else if (!std::strcmp(argv[i], "-mass_car"))
		{
			mass_car = std::atof(argv[i + 1]);
		}
		else
		{
			std::cout << "unrecognised Option: " << argv[i] << std::endl;
			std::cout << "use -help to see all available options" << std::endl;
			return 0;
		}
	}
	
	std::vector<rigidbody*> objects;

	objects.push_back(new vehicle(vec3d(1, 1, 1)));
	for (int i = 0; i < n_sphere; ++i)
	{
		objects.push_back(new sphere(vec3d(0, 0, 0)));
	}

	simulation sim = simulation(objects, objects.size()); //how to do this (n_objects is changing at runtime so no array possible)->use iterators
	for (int i = 0; i < 2; i++)
	{
		sim.step(objects, 0.001);
	}

	//TutorialApplication rend;
	//rend.go();
	int success=drawframe();
	return 0;
}