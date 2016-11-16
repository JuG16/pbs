#pragma once

#include "../include/ogre/build/sdk/include/OGRE/OgreAnimable.h" //instead of last part put <"sometext"> from API
#include "sphere.h"
#include "typedef.h"

template <typename container>
void drawframe(container &spheres, vehicle &car)
{
	//do plotting using ogre3d
	//sphere is a container with random access "[]"
	//every element is of type "sphere"
}