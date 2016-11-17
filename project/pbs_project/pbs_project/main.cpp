#include "typedef.h"
#include "simulation.h"
#include "rendering.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>

int main(int argc, char *argv)
{
	//vectors for options and corresponding descriptions for -help
	const std::vector<std::string> options (std::string("n_sphere"), std::string("mass_sphere"), std::string("radius_sphere"), std::string("mass_car")); //lookup how to do
	const std::vector<std::string> description = ();

	//standard settings
	int_t n_sphere = 10;
	real_t mass_sphere = 1;
	real_t radius_sphere = 0.1;
	real_t mass_car = 100;
	if (argc == 1&&argv[0]=="-help") //how to do this
	{
		std::cout << "Usage: -\"option1\" <value1> -\"option2\" <value2> ..." << std::endl;
		std::cout << std::setw(15)<<"Options" << "Description"<< std::endl;
		for (int i = 0; i < options.size(); ++i)
		{
			std::cout << std::setw(15) << options.at(i) << description.at(i) << std::endl;
		}
		return 0;
	}
	if (argc % 2 != 0)
	{
		std::cout << "Usage: -\"option\" <value>" << std::endl;
		std::cout << "Type ./main -help for available options" << std::endl;
		return 0;
	}
	for (int i = 0; i < argc; i+=2)
	{
		//do some intelligent (not writing everything explicitly) switching to set correct values
	}

	std::vector<sphere> spheres;
	for (int i = 0; i < n_sphere; ++i)
	{
		spheres.push_back(/*initialize sphere !position!*/);
	}
	return 0;
}