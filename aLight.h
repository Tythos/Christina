/*
 *  aLight.h
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 8/15/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#ifndef ARTEMIS_LIGHT_H
#define ARTEMIS_LIGHT_H

#include "aObject.h"
#include "kQuat.h"
#include "aMesh.h"

class aLight {
private:
	kQuat * position;	// Defined within parent frame; global frame by default
	kQuat * ambient;	// Color of ambient (non-directional) light
	kQuat * diffuse;	// Color of diffuse (directional) light
	aLight * next;
	float attenuation;	// Light attenuation factor
	int lightId;
	bool isEnabled;		// Lights are disabled by default
	bool isDirectional; // Directional lights produce parallel rays in the _position direction; non-directional lights are positional, where _position specifies the light location in the world
	bool isVisible;		// Visible lights are rendered as small points with the light's ambient color
	aMesh * lightObject;
protected:
public:
	// Constructors
	aLight();
	~aLight();
	
	// Access
	void enable();
	void disable();
	bool getStatus() { return isEnabled; }
	bool getVisible() { return isVisible; }
	void setVisible(bool v) { isVisible = v; }
	
	// Lighting
	void setAmbient(float r, float g, float b);
	void setDiffuse(float r, float g, float b);
	void setPosition(float x, float y, float z);
	void setAttenuation(float a) { attenuation = a; }
	void setDirectional() { isDirectional = true; }
	void setPositional() { isDirectional = false; }
	void refreshPosition();
	
	// List methods
	aLight * get(int n); // n < 0 returns first; n > length returns last
	void set(aLight * value, int n); // n > length and n < 0 sets last
	int listLength();
	
	int getGLLightNum(int n);
};

#endif