#pragma once

#include "../include/ogre/build/sdk/include/OGRE/Ogre.h" //instead of last part put <"sometext"> from API
#include "sphere.h"
#include "typedef.h"
#include "../include/ogre/OgreMain/include/OgreSceneManager.h"
#include "../include/ogre/OgreMain/include/OgreRoot.h"




//template <typename container>
//void drawframe(container &spheres, vehicle &car)
void drawframe()
{
	//do plotting using ogre3d
	//sphere is a container with random access "[]"
	//every element is of type "sphere"
	Ogre::SceneManager* mSceneMgr= createSceneManager(Ogre::ST_GENERIC);
	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
	Ogre::Entity* ogreEntity = mSceneMgr->createEntity("ogrehead.mesh");
	Ogre::SceneNode* ogreNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	ogreNode->attachObject(ogreEntity);
	Ogre::Light* light;
	light->setPosition(20, 80, 50);
}