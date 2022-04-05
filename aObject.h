/*
 *  aObject.h
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 10/8/08.
 *  Copyright 2008 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#ifndef ARTEMIS_OBJECT_H
#define ARTEMIS_OBJECT_H

#include <stdio.h>
#include "kString.h"
#include "kTrans.h"
#include "kQuat.h"
#include "aColor.h"
#include "aMesh.h"
#include "aTexture.h"
#include "SDL.h"

class aObject {
private:
protected:
	kTrans * frame;
	kQuat * linearVelocity; // Defined in parent frame
	kQuat * angularVelocity; // Defined in parent frame, valued quaternion rotation
	aObject * parent;
	aObject * children;
	aObject * next;
	aColor * ambientMaterial;
	aColor * diffuseMaterial;
	aTexture * tex;
	aMesh * mesh;
public:
	// Data
	bool isVisible;

	// Constructors
	aObject(aObject * p = NULL);
	aObject(float x, float y, float z, aObject * p = NULL);
	aObject(kQuat orientation, aObject * p = NULL);
	aObject(float x, float y, float z, kQuat orientation, aObject * p = NULL);
	~aObject();
	void freeChildren();
	
	// Access
	kTrans getFrame();
	kQuat getLocation();
	kQuat getRotation();
	kQuat getLinearVelocity();
	kQuat getAngularVelocity();
	kQuat getScaling();
	
	// Color
	void setAmbient(float r, float g, float b);
	void setDiffuse(float r, float g, float b);
	
	void setTexture(kString filename);
	void setTexture(SDL_Surface * surf);
	void setScaling(float x, float y, float z);
	// TEMPORARY, FOR DEMO; end class will allow manipulation only through physics forces
	void setPosition(float x, float y, float z);
	void setRotation(float w, float x, float y, float z);
	void setLinearVelocity(float x, float y, float z);
	void setAngularVelocity(float w, float x, float y, float z);
		
	// Methods
	virtual void update(float dt, bool updateChildren = true);
	virtual void render(bool renderChildren = true);
	
	// Child manipulation
	void addChild(aObject * newChild);
	
	// Listing
	aObject * get(int n); // n < 0 returns first; n > length returns last
	void set(aObject * value, int n); // n > length and n < 0 sets last
	int listLength();
};

#endif