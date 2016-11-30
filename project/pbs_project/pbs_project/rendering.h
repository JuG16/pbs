#pragma once

//#include "../include/ogre/build/sdk/include/OGRE/Ogre.h" //instead of last part put <"sometext"> from API
#include "sphere.h"
#include "vehicle.h"
#include "plane.h"
#include "typedef.h"
//#include "BaseApplication.h"

#include <irrlicht.h>


//#include "BaseApplication.h"

//---------------------------------------------------------------------------

/*class TutorialApplication : public BaseApplication
{
public:
	TutorialApplication(void)
	{

	}
	virtual ~TutorialApplication(void)
	{

	}

protected:
	void createScene(void)
	{
		mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
		Ogre::Entity* ogreEntity = mSceneMgr->createEntity("sphere.mesh");
		Ogre::SceneNode* ogreNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		ogreNode->attachObject(ogreEntity);
		Ogre::Light* light = mSceneMgr->createLight("MainLight");
		light->setPosition(20, 80, 50);
	}
};


template <typename container>
//void drawframe(container &spheres, vehicle &car)
void drawframe()
{
	//do plotting using ogre3d
	//sphere is a container with random access "[]"
	//every element is of type "sphere"
	Ogre::Root *root = new Ogre::Root();
	Ogre::SceneManager* mSceneMgr=root->createSceneManager(Ogre::ST_GENERIC);
	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
	Ogre::Entity* ogreEntity = mSceneMgr->createEntity("ogrehead.mesh");
	Ogre::SceneNode* ogreNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	ogreNode->attachObject(ogreEntity);
	Ogre::Light* light=new Ogre::Light();
	light->setPosition(20, 80, 50);
}*/

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

#ifdef _IRR_WINDOWS_
#pragma comment(lib, "Irrlicht.lib")
//#pragma comment(linker, "/subsystem:windows /ENTRY:mainCRTStartup")
#endif
int drawframe() {
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

	float TRadius = 5.0f;
	scene::ISceneNode *Node = smgr->addSphereSceneNode(TRadius, 32);
	Node->setMaterialFlag(video::EMF_LIGHTING, 1);
	Node->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
	Node->setMaterialTexture(0, driver->getTexture("../media/stones.jpg"));


	smgr->addCameraSceneNodeMaya();
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