/*
 *  aCube.h
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 7/12/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#ifndef ARTEMIS_CUBE_H
#define ARTEMIS_CUBE_H

#include "aObject.h"
#include <SDL_opengl.h>

class aCube : public aObject {
private:
	float size;
protected:
public:
	// Structors
	aCube();
	virtual ~aCube();

	// Accessors
	float getSize() { return size; }
	void setSize(float v);
};

#endif