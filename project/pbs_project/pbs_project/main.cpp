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
	const std::vector<std::string> options({ "n_sphere","mass_sphere","radius_sphere","mass_car", "fps", "endtime" });
	const std::vector<std::string> description ({ "Sets the number of spheres in the box", "sets the mass of every single sphere", "sets the radius of every single sphere", "sets the mass of the car", "frames per second (not relevant for realtime)", "set how long the simulation should run (not relevant for realtime)" });


	const real_t dt = 0.001;

	//standard settings
	int_t n_sphere = 100;
	real_t mass_sphere = 1;
	real_t radius_sphere = 1; //0.1
	real_t mass_car = 100;
	int_t fps = 30;
	real_t endtime = 10;
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
		else if (!std::strcmp(argv[i], "-fps"))
		{
			fps = std::atoi(argv[i + 1]);
		}
		else if (!std::strcmp(argv[i], "-endtime"))
		{
			endtime = std::atof(argv[i + 1]);
		}
		else
		{
			std::cout << "unrecognised Option: " << argv[i] << std::endl;
			std::cout << "use -help to see all available options" << std::endl;
			return 0;
		}
	}
	
	std::vector<rigidbody*> objects;

	//objects.push_back(new vehicle(vec3d(20, 20, 20)));
	objects.push_back(new vehicle(vec3d(10, 5, 0), 1000*Eigen::MatrixXd::Identity(3, 3), 100, 5, 5, 5));
	//objects.push_back(new sphere(vec3d(0, 0, 0)));
	const int x_grid = 4;
	const int y_grid = 5;
	const int z_grid = 5;
	const real_t diameter = 2 * radius_sphere;
	for (int i = 0; i < x_grid; i++){
		for (int j = 0; j < y_grid; j++) {
			for (int k = 0; k < z_grid; k++) {
				objects.push_back(new sphere(vec3d(i*diameter, j*diameter,k*diameter)));
			}
		}
	}

	//objects.push_back(new sphere(vec3d(-10, 0, 0), Eigen::MatrixXd::Identity(3, 3), 5, 1, vec3d(100,0,0)));
	//objects.push_back(new sphere(vec3d(10, 0, 0), Eigen::MatrixXd::Identity(3, 3), 5, 1, vec3d(-100,0,0)));
	
	/*objects.push_back(new sphere(vec3d(-10, 0, 0), Eigen::MatrixXd::Identity(3, 3), 5, 1, vec3d(100, 0, 0)));
	objects.push_back(new vehicle(vec3d(10, 0, 0), Eigen::MatrixXd::Identity(3, 3), 1, 5, 5, 5, vec3d(-100, 0, 0)));
	//objects[1]->setstatic();*/

#ifndef REALTIME
	simulation sim = simulation(objects, objects.size()); //how to do this (n_objects is changing at runtime so no array possible)->use iterators
	std::vector<std::vector<renderdata>> data;
	//simulation part
	real_t currtime = 0;
	const int_t steps = endtime / dt;
	const int_t drawsteps = 1. / dt / fps; //determines the regularity when steps get plotted
	const int_t perccount = steps / 20;
	std::cout << "starting simulation" << std::endl;
	for (int i = 0; i < steps; ++i)
	{
		sim.step(objects, dt);
		if (i%drawsteps == 0)
		{
			std::vector<renderdata> temp; //inefficient
			for (auto object : objects)
			{
				temp.emplace_back(object, object->getpos(), object->getquat());
			}
			data.push_back(temp);
		}
		if (i%perccount == 0)
		{
			std::cout << 5 * (i / perccount) << "% complete" << std::endl;
		}
	}
	std::cout << "computation completed! type 1 to continue." << std::endl;
	std::cout << data.size() << std::endl;
	int_t continuetest;
	std::cin >> continuetest;

	//rendering part

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

	std::chrono::high_resolution_clock clock;
	std::chrono::time_point<std::chrono::high_resolution_clock> t_start;
	std::chrono::time_point<std::chrono::high_resolution_clock> t_end;
	while (true)
	{
		for (int_t i = 0; i < data.size(); ++i)
		{

			//plot and sleep
			t_start = clock.now();
			smgr->clear();
			for (auto renderdat : data[i])
			{
				renderdat.draw(smgr, driver);
			}

			/*driver->setTextureCreationFlag(video::ETCF_CREATE_MIP_MAPS, false);

			smgr->addSkyBoxSceneNode(
				driver->getTexture("../media/irrlicht2_up.jpg"),
				driver->getTexture("../media/irrlicht2_dn.jpg"),
				driver->getTexture("../media/irrlicht2_lf.jpg"),
				driver->getTexture("../media/irrlicht2_rt.jpg"),
				driver->getTexture("../media/irrlicht2_ft.jpg"),
				driver->getTexture("../media/irrlicht2_bk.jpg"));

			driver->setTextureCreationFlag(video::ETCF_CREATE_MIP_MAPS, true);

			// add a camera and disable the mouse cursor
			scene::ICameraSceneNode* cam = smgr->addCameraSceneNodeFPS();
			cam->setPosition(core::vector3df(0, 30, -40));
			cam->setTarget(core::vector3df(0, 5, 0));
			device->getCursorControl()->setVisible(false);*/

			smgr->addSkyDomeSceneNode(driver->getTexture("../media/skydome.jpg"), 240, 240, 1.0f, 2.0f);

			smgr->addCameraSceneNode(0, vector3df(0, 3, -4), vector3df(0, 5, 0));
			driver->beginScene(true, true, SColor(255, 255, 255, 255));

			smgr->drawAll();
			guienv->drawAll();

			driver->endScene();
			t_end = clock.now();
			if (std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count() < 1000. / fps)
			{
				std::cout << "sleeping" << std::endl;
				const unsigned sleep_t = 1000. / fps - std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
				std::this_thread::sleep_for(std::chrono::milliseconds(sleep_t));
			}
		}
		bool exitsim = false;
		while (true)
		{
			std::cout << "type 0 to exit" << std::endl;
			std::cout << "type 1 to repeat" << std::endl;
			int_t temp;
			std::cin >> temp;
			if (temp == 0)
			{
				exitsim = true;
				break;
			}
			else if (temp == 1)
			{
				break;
			}
			else
			{
				std::cout << "unrecognised input." << std::endl;
			}
		}
		if (exitsim)
		{
			break;
		}

	}
	device->drop();
#endif

#ifdef REALTIME
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
		if (std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count() < 100*dt)
		{
			std::cout << "sleeping" << std::endl;
			const unsigned sleep_t = 100 * dt - std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
			std::this_thread::sleep_for(std::chrono::milliseconds(sleep_t));
		}
	}
	device->drop();
#endif

	return 0;
}