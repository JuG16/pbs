#pragma once


#include "sphere.h"
#include "vehicle.h"
#include "plane.h"
#include "typedef.h"



#ifdef _IRR_WINDOWS_
#pragma comment(lib, "Irrlicht.lib")
//#pragma comment(linker, "/subsystem:windows /ENTRY:mainCRTStartup")
#endif
int drawframe(std::vector<rigidbody*> objects, int_t n_objects) {
	IrrlichtDevice *device =
		createDevice(video::EDT_SOFTWARE, dimension2d<u32>(640, 480), 16,
			false, false, false, 0);

	if (!device)
		return 1;
	device->setWindowCaption(L"Hello World! - Irrlicht Engine Demo");
	IVideoDriver* driver = device->getVideoDriver();
	ISceneManager* smgr = device->getSceneManager();
	IGUIEnvironment* guienv = device->getGUIEnvironment();
	guienv->addStaticText(L"Hello World! This is the Irrlicht Software renderer!",
		rect<s32>(10, 10, 260, 22), true);

	/*IAnimatedMesh* mesh = smgr->getMesh("../media/sydney.md2");
	if (!mesh)
	{
		device->drop();
		return 1;
	}
	IAnimatedMeshSceneNode* node = smgr->addAnimatedMeshSceneNode(mesh);
	if (node)
	{
		node->setMaterialFlag(EMF_LIGHTING, false);
		node->setMD2Animation(scene::EMAT_STAND);
		node->setMaterialTexture(0, driver->getTexture("../media/sydney.bmp"));		
	}*/

	/*float TRadius = 3.0f;
	scene::ISceneNode *Node = smgr->addSphereSceneNode(TRadius, 32);
	Node->setMaterialFlag(video::EMF_LIGHTING, 1);
	Node->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
	Node->setMaterialTexture(0, driver->getTexture("../media/stones.jpg"));
	
	vector3df TScale = vector3df(10.0f, 0.5f, 10.0f);
	scene::ISceneNode *NodeBox = smgr->addCubeSceneNode(1.0f);
	NodeBox->setScale(TScale);
	NodeBox->setMaterialFlag(video::EMF_LIGHTING, 1);
	NodeBox->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
	NodeBox->setMaterialTexture(0, driver->getTexture("../media/wall.jpg"));*/

	for (int i = 0; i < n_objects; i++) {
		
		objects[i]->addtoscene(smgr, driver);
	}

	smgr->addCameraSceneNodeMaya();
	//smgr->addCameraSceneNode(0, vector3df(0, 30, -40), vector3df(0, 5, 0));
	while (device->run())
	{
		driver->beginScene(true, true, SColor(255, 100, 101, 140));

		smgr->drawAll();
		guienv->drawAll();

		driver->endScene();
	}
	device->drop();

	return 0;
}