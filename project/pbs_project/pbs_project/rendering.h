#pragma once

#include "../include/ogre/build/sdk/include/OGRE/Ogre.h" //instead of last part put <"sometext"> from API
#include "sphere.h"
#include "vehicle.h"
#include "plane.h"
#include "typedef.h"
#include "BaseApplication.h"



#include "BaseApplication.h"

//---------------------------------------------------------------------------

class TutorialApplication : public BaseApplication
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


//template <typename container>
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
}