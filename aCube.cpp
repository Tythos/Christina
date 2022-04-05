/*
 *  aCube.cpp
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 7/12/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#include <SDL.h>
#include <SDL_opengl.h>
#include <SDL_image.h>

#include "aCube.h"
#include "aObject.h"
#include "numericMath.h"

aCube::aCube()
: aObject(NULL) {
	size = 1.0f;
	mesh = new aMesh();
	mesh->loadCube(size);
	tex = new aTexture();
	tex->loadFromFile("brick.png");
}

aCube::~aCube() {
	// Use default object constructor
}

void aCube::setSize(float v) {
	size = v;
	mesh->loadCube(size);
}