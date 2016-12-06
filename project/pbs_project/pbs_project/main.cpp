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
#include <chrono>
#include <thread>

#include "gjk_algorithm.h"


int main(int argc, char** argv)
{
	//vectors for options and corresponding descriptions for -help
	const std::vector<std::string> options({ "n_sphere","mass_sphere","radius_sphere","mass_car" });
	const std::vector<std::string> description ({ "Sets the number of spheres in the box", "sets the mass of every single sphere", "sets the radius of every single sphere", "sets the mass of the car" });


	//standard settings
	int_t n_sphere = 12;
	real_t mass_sphere = 1;
	real_t radius_sphere = 3; //0.1
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

	objects.push_back(new vehicle(vec3d(-10, -10, -10)));
	//objects.push_back(new vehicle(vec3d(10, 0, 0), Eigen::MatrixXd::Identity(3, 3), 100, 10, 10, 10));
	//objects.push_back(new sphere(vec3d(0, 0, 0)));
	const int x_grid = 2;
	const int y_grid = 2;
	const int z_grid = 3;
	const int diameter = 2 * radius_sphere;
	for (int i = 0; i < x_grid; i++){
		for (int j = 0; j < y_grid; j++) {
			for (int k = 0; k < z_grid; k++) {
				objects.push_back(new sphere(vec3d(i*diameter, j*diameter,k*diameter)));
			}
		}
	}
	//objects.push_back(new sphere(vec3d(-10, 0, 0), Eigen::MatrixXd::Identity(3, 3), 5, 1, vec3d(100,0,0)));
	//objects.push_back(new sphere(vec3d(10, 0, 0), Eigen::MatrixXd::Identity(3, 3), 5, 1, vec3d(-100,0,0)));

	/*vec3d contactpoint;
	vec3d n;
	real_t pen_depth;
	gjk_algorithm gjk = gjk_algorithm();
	std::cout << "check" << std::endl;
	if (gjk.collisiondetection(objects[0], objects[1]))
	{
		std::cout << "returned true" << std::endl;
	}*/
	
	IrrlichtDevice *device =
		createDevice(video::EDT_SOFTWARE, dimension2d<u32>(640, 480), 16,
			false, false, false, 0);

	if (!device)
		return 1;
	device->setWindowCaption(L"Physically based Simulation - Project");
	IVideoDriver* driver = device->getVideoDriver();
	ISceneManager* smgr = device->getSceneManager();
	IGUIEnvironment* guienv = device->getGUIEnvironment();
	guienv->addStaticText(L"Our rollercoaster!",
		rect<s32>(10, 10, 260, 22), true);
	
	for (int i = 0; i < objects.size(); i++) {

		objects[i]->addtoscene(smgr, driver);
	}
	smgr->addCameraSceneNode(0, vector3df(0, -30, 0), vector3df(0, 5, 0));
	driver->beginScene(true, true, SColor(255, 100, 101, 140));

	smgr->drawAll();
	guienv->drawAll();

	driver->endScene();

	//int success = drawframe(objects, objects.size());
	simulation sim = simulation(objects, objects.size()); //how to do this (n_objects is changing at runtime so no array possible)->use iterators
	const real_t dt = 0.001;
	std::chrono::high_resolution_clock clock;
	std::chrono::time_point<std::chrono::high_resolution_clock> t_start;
	std::chrono::time_point<std::chrono::high_resolution_clock> t_end;

	while (device->run())
	{
		t_start = clock.now();
		smgr->clear();
		sim.step(objects, dt);
		for (int i = 0; i < objects.size(); i++) {

			objects[i]->addtoscene(smgr, driver);
		}

		//smgr->addCameraSceneNodeMaya();
		smgr->addCameraSceneNode(0, vector3df(0, 30, -40), vector3df(0, 5, 0));
		driver->beginScene(true, true, SColor(255, 100, 101, 140));

		smgr->drawAll();
		guienv->drawAll();

		driver->endScene();
		t_end = clock.now();
		if (std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count() < 10000*dt)
		{
			std::cout << "sleeping" << std::endl;
			const unsigned sleep_t = 10000 * dt - std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
			std::this_thread::sleep_for(std::chrono::milliseconds(sleep_t));
		}
	}
	device->drop();
	return 0;
}