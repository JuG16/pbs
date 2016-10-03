//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//=============================================================================

#pragma once

#include "Utilities/Vector2T.h"
#include <stdlib.h>

class MPoint
{
public:
	Vec2 pos;
	bool fixed;

public:
	MPoint(double x, double y) { fixed = false; pos.x() = x; pos.y() = y; }
	MPoint(void) { fixed = false; pos.x() = 0; pos.y() = 0; }
	~MPoint(void){}

	void render();
};

class MSpring
{
public:
	MPoint *a, *b;
	
public:
	MSpring(void) : a(nullptr), b(nullptr) {}
	~MSpring(void) {}
	
	void set(MPoint *_a, MPoint *_b) { a = _a; b = _b; }
	void render();
};
