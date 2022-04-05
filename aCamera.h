/*
 *  aCamera.h
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 4/19/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#ifndef ARTEMIS_CAMERA_H
#define ARTEMIS_CAMERA_H

#include "kQuat.h"
#include "aSkybox.h"

class aCamera {
private:
	kQuat * position;
	kQuat * target;
	kQuat * up;
	float fieldAngle;
	float aspectRatio;
	float nearClip;
	float farClip;
	float singularityTolerance;
protected:
public:
	aCamera();
	~aCamera();
	aSkybox * skybox;
	
	// Set accessors
	void setPosition(float x, float y, float z);
	void setTarget(float x, float y, float z);
	void setUp(float x, float y, float z);
	void setRight(float x, float y, float z);
	void setFieldAngle(float v) { fieldAngle = v; }
	void setAspectRatio(float v) { aspectRatio = v; }
	void setNearClip(float v) { nearClip = v; }
	void setFarClip(float v);

	// Camera accessors
	float getFieldAngle() { return fieldAngle; }
	float getAspectRatio() { return aspectRatio; }
	float getNearClip() { return nearClip; }
	float getFarClip() { return farClip; }
	
	// Spacial accessors
	kQuat * getPosition() { return position; }
	kQuat * getTarget() { return target; }
	kQuat * getUp() { return up; }
	
	void applyCamera();
	void print();

	// Skybox (background) settings
	void setSkyboxTexture(kString filename);
	void setSkyboxTexture(SDL_Surface * surf);
	void setSkyboxResolution(int n);
	void renderSkybox();

	// Movement functions
	void rotateFocus(float dTht, float dPhi);
	void rotateCenter(float dTht, float dPhi);
	void pan(float dx, float dy, float dz);
	void focus(float x, float y, float z);
	void zoomFocus(float dz);
};

#endif