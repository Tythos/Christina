/*
 *  aLight.cpp
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 8/15/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#include "aLight.h"
#include <SDL_opengl.h>
#include <stdio.h>

aLight::aLight() {
	static int lightCount = 0;
	position = new kQuat(0.0f, 0.0f, 10.0f, 0.0f);
	ambient = new kQuat(1.0f, 0.4f, 0.4f, 0.4f);
	diffuse = new kQuat(1.0f, 0.8f, 0.8f, 0.8f);
	next = NULL;
	attenuation = 0.2f;
	lightId = lightCount;
	lightCount++;
	isEnabled = false;
	isDirectional = true;
	isVisible = false;
	lightObject = new aMesh();
	lightObject->loadSphere(0.1f, 32);
	lightObject->disableTexture();
	printf("Light %d created!\n", lightId);
}

aLight::~aLight() {
	delete position;
	delete ambient;
	delete diffuse;
}

// Access
void aLight::enable() {
	// Make sure lighting is enabled and configured
	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glFrontFace(GL_CCW);

	// Turn on light!
	isEnabled = true;
	int id = getGLLightNum(lightId);
	float ambientVals[] = { ambient->getVecI(), ambient->getVecJ(), ambient->getVecK(), ambient->getScl() };
	float diffuseVals[] = { diffuse->getVecI(), diffuse->getVecJ(), diffuse->getVecK(), diffuse->getScl() };
	float positionVals[4] = { position->getVecI(), position->getVecJ(), position->getVecK(), 1.0f };
	if (isDirectional) {
		positionVals[3] = 1.0f;
	} else {
		positionVals[3] = 0.0f;
	}
	glLightfv(id, GL_AMBIENT, ambientVals);
	glLightfv(id, GL_DIFFUSE, diffuseVals);
	glLightfv(id, GL_POSITION, positionVals);
	glLightf(id, GL_LINEAR_ATTENUATION, attenuation);	
	glEnable(id);
}

void aLight::disable() {
	glDisable(getGLLightNum(lightId));
	isEnabled = false;
}

// Lighting
void aLight::setAmbient(float r, float g, float b) {
	ambient->set(1.0f, r, g, b);
}

void aLight::setDiffuse(float r, float g, float b) {
	diffuse->set(1.0f, r, g, b);
}

void aLight::setPosition(float x, float y, float z) {
	position->set(1.0f, x, y, z);
}

void aLight::refreshPosition() {
	// Update light position
	int id = getGLLightNum(lightId);
	float positionVals[4] = { position->getVecI(), position->getVecJ(), position->getVecK(), 1.0f };
	if (isDirectional) {
		positionVals[3] = 1.0f;
	} else {
		positionVals[3] = 0.0f;
	}
	glLightfv(id, GL_POSITION, positionVals);

	// Draw light?
	if (isVisible) {
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glPushMatrix(); {
			glTranslatef(positionVals[0], positionVals[1], positionVals[2]);
			glColor4f(ambient->getVecI(), ambient->getVecK(), ambient->getVecK(), 1.0f);
			float colorVals[4] = { ambient->getVecI(), ambient->getVecJ(), ambient->getVecK(), 1.0f };
			float bbodyVals[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
			glMaterialfv(GL_FRONT, GL_AMBIENT, colorVals);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, bbodyVals);
			lightObject->render();
		} glPopMatrix();
	}
}

// List methods
aLight * aLight::get(int n) {
	// n < 0 returns first; n > length returns last
	if (n >= GL_MAX_LIGHTS) {
		n = GL_MAX_LIGHTS - 1;
	}
	if (n <= 0 || next == NULL) {
		return this;
	} else {
		return next->get(n-1);
	}	
}

void aLight::set(aLight * value, int n) {	
	// n > length and n < 0 sets last
	if (n >= GL_MAX_LIGHTS) {
		n = GL_MAX_LIGHTS - 1;
	}
	if (n == 0 || next == NULL) {
		next = value;
	} else if (n < 0 && (next == NULL || !getStatus())) {
		next = value;
	} else {
		next->set(value, n-1);
	}		
}

int aLight::listLength() {
	if (this == NULL) {
		return 0;
	} else if (next == NULL) {
		return 1;
	} else {
		return next->listLength() + 1;
	}
}
	
int aLight::getGLLightNum(int n) {
	int zero = (int)GL_LIGHT0;
	return zero + n;
}