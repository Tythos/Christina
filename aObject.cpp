/*
 *  aObject.cpp
 *  Artemis
 *
 *  Created by Brian Kirkpatrick on 10/8/08.
 *  Copyright 2008 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#include "aObject.h"
#include <SDL_opengl.h>

aObject::aObject(aObject * p) {
	frame = new kTrans();
	linearVelocity = new kQuat(1.0f, 0.0f, 0.0f, 0.0f);
	angularVelocity = new kQuat(0.0f, 1.0f, 0.0f, 0.0f);
	parent = p;
	children = NULL;
	next = NULL;
	ambientMaterial = new aColor(0.7f, 0.7f, 0.7f, 1.0f);
	diffuseMaterial = new aColor(0.9f, 0.9f, 0.7f, 1.0f);
	tex = NULL;
	mesh = NULL;
	isVisible = true;
}

aObject::aObject(float x, float y, float z, aObject * p) {
	frame = new kTrans(); frame->setPos(x, y, z);
	linearVelocity = new kQuat(1.0f, 0.0f, 0.0f, 0.0f);
	angularVelocity = new kQuat(0.0f, 1.0f, 0.0f, 0.0f);
	parent = p;
	children = NULL;
	next = NULL;
	ambientMaterial = new aColor(0.7f, 0.7f, 0.7f, 1.0f);
	diffuseMaterial = new aColor(0.9f, 0.9f, 0.7f, 1.0f);
	mesh = NULL;
	tex = NULL;
	isVisible = true;
}

aObject::aObject(kQuat orientation, aObject * p) {
	// Note: orientation is a valued rotation quaternion
	frame = new kTrans();
	frame->setRot(orientation.getScl(), orientation.getVecI(), orientation.getVecJ(), orientation.getVecK());
	linearVelocity = new kQuat(1.0f, 0.0f, 0.0f, 0.0f);
	angularVelocity = new kQuat(0.0f, 1.0f, 0.0f, 0.0f);
	parent = p;
	children = NULL;
	next = NULL;
	ambientMaterial = new aColor(0.7f, 0.7f, 0.7f, 1.0f);
	diffuseMaterial = new aColor(0.9f, 0.9f, 0.7f, 1.0f);
	mesh = NULL;
	tex = NULL;
	isVisible = true;
}

aObject::aObject(float x, float y, float z, kQuat orientation, aObject * p) {
	// Note: orietnation is a valued rotation quaternion
	frame = new kTrans();
	frame->setPos(x, y, z);
	frame->setRot(orientation.getScl(), orientation.getVecI(), orientation.getVecJ(), orientation.getVecK());
	linearVelocity = new kQuat(1.0f, 0.0f, 0.0f, 0.0f);
	angularVelocity = new kQuat(0.0f, 1.0f, 0.0f, 0.0f);
	parent = p;
	children = NULL;
	next = NULL;
	ambientMaterial = new aColor(0.7f, 0.7f, 0.7f, 1.0f);
	diffuseMaterial = new aColor(0.9f, 0.9f, 0.7f, 1.0f);
	mesh = NULL;
	tex = NULL;
	isVisible = true;
}

aObject::~aObject() {
	delete frame;
	delete linearVelocity;
	delete angularVelocity;
	delete ambientMaterial;
	delete diffuseMaterial;
	if (mesh != NULL) { delete mesh; }
	if (tex != NULL) { delete tex; }
}

void aObject::freeChildren() {
	// Before an aObject is destroyed, it may need to free its children
	int childCount = children->listLength();
	aObject * tmp = NULL;
	for (int i = childCount; i > 0; i--) {
		tmp = children->get(i-1);
		tmp->freeChildren();
		delete tmp;
	}
	children = NULL;
}

kTrans aObject::getFrame() {
	kTrans toReturn = (*frame);
	return toReturn;
}

kQuat aObject::getLocation() {
	kQuat toReturn = frame->getPos();
	return toReturn;
}

kQuat aObject::getRotation() {
	kQuat toReturn = frame->getRot();
	return toReturn;
}

kQuat aObject::getScaling() {
	kQuat toReturn = frame->getScl();
	return toReturn;
}

void aObject::setAmbient(float r, float g, float b) {
	ambientMaterial->set(r, g, b, 1.0f);
}

void aObject::setDiffuse(float r, float g, float b) {
	diffuseMaterial->set(r, g, b, 1.0f);
}

void aObject::setTexture(kString filename) {
	if (tex->isLoaded()) { tex->unload(); }
	tex->loadFromFile(filename);
}

void aObject::setTexture(SDL_Surface * surf) {
	if (tex->isLoaded()) { tex->unload(); }
	tex->loadFromSurface(surf);
}

void aObject::setScaling(float x, float y, float z) {
	frame->setScl(x, y, z);
}

kQuat aObject::getLinearVelocity() {
	kQuat toReturn = (*linearVelocity);
	return toReturn;
}

kQuat aObject::getAngularVelocity() {
	kQuat toReturn = (*angularVelocity);
	return toReturn;
}

void aObject::update(float dt, bool updateChildren) {
	// Update object position based on current velocity and given timestep
	// Follow up by updating child positions, unless indicated otherwise
	frame->update(linearVelocity, angularVelocity, dt);
	if (updateChildren && children != NULL) {
		for (int i = 0; i < children->listLength(); i++) {
			children->get(i)->update(dt, updateChildren);
		}
	}
}

void aObject::render(bool renderChildren) {
	// For base object, draw orthonormal coordinates
	frame->glApply(); {
		if (isVisible) {
			if (mesh == NULL) {
				float red[] = { 1.0f, 0.0f, 0.0f, 1.0f };
				float green[] = { 0.0f, 1.0f, 0.0f, 1.0f };
				float blue[] = { 0.0f, 0.0f, 1.0f, 1.0f };
				float black[] = { 0.0f, 0.0f, 0.0f, 1.0f };
				glMaterialfv(GL_FRONT, GL_DIFFUSE, black);
				glBegin(GL_LINES); {
					glMaterialfv(GL_FRONT, GL_AMBIENT, red);
					glColor3f(1.0f, 0.0f, 0.0f);
					glVertex3f(0.0f, 0.0f, 0.0f);
					glVertex3f(1.0f, 0.0f, 0.0f);
					glMaterialfv(GL_FRONT, GL_AMBIENT, green);
					glColor3f(0.0f, 1.0f, 0.0f);
					glVertex3f(0.0f, 0.0f, 0.0f);
					glVertex3f(0.0f, 1.0f, 0.0f);
					glMaterialfv(GL_FRONT, GL_AMBIENT, blue);
					glColor3f(0.0f, 0.0f, 1.0f);
					glVertex3f(0.0f, 0.0f, 0.0f);
					glVertex3f(0.0f, 0.0f, 1.0f);
				} glEnd();
			} else {
				// Set material, texture
				diffuseMaterial->setDiffuseMat();
				ambientMaterial->setAmbientMat();
				if (tex != NULL) { tex->bind(); }
				mesh->render();
				if (tex != NULL) { tex->unbind(); }
			}
		}
		
		// Render children as well?
		if (renderChildren && children != NULL) {
			for (int i = 0; i < children->listLength(); i++) {
				children->get(i)->render(renderChildren);
			}
		}
	} frame->glUnapply();
}

void aObject::setPosition(float x, float y, float z) {
	frame->setPos(x, y, z);
}

void aObject::setRotation(float w, float x, float y, float z) {
	frame->setRot(w, x, y, z);
}

void aObject::setLinearVelocity(float x, float y, float z) {
	linearVelocity->set(0.0f, x, y, z);
}

void aObject::setAngularVelocity(float w, float x, float y, float z) {
	angularVelocity->set(w, x, y, z);
}

void aObject::addChild(aObject * newChild) {
	if (children == NULL) {
		children = newChild;
	} else {
		children->set(newChild, -1);
	}
}

aObject * aObject::get(int n) {
	if (n <= 0 || next == NULL) {
		return this;
	} else {
		return next->get(n-1);
	}
}

void aObject::set(aObject * value, int n) {
	if (n == 0 || next == NULL) {
		next = value;
	} else if (n < 0 && next == NULL) {
		next = value;
	} else {
		next->set(value, n-1);
	}		
}

int aObject::listLength() {
	if (this == NULL) {
		return 0;
	} else if (next == NULL) {
		return 1;
	} else {
		return next->listLength() + 1;
	}
}