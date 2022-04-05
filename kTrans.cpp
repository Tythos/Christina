/*
 *  kTrans.cpp
 *  kQuat
 *
 *  Created by Brian Kirkpatrick on 7/9/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

// Necessary for Win32 math.h to expose math constant definitions (M_PI, etc)
#define _USE_MATH_DEFINES
#include <math.h>

#include <stdio.h>
#include "SDL.h"
#include <SDL_opengl.h>
#include "kQuat.h"
#include "kTrans.h"

kTrans::kTrans() {
	_pos = new kQuat(0.0f, 0.0f, 0.0f, 0.0f);
	_rot = new kQuat(0.0f, 1.0f, 0.0f, 0.0f);
	_scl = new kQuat(0.0f, 1.0f, 1.0f, 1.0f);
}

kTrans::~kTrans() {
	delete _pos;
	delete _rot;
	delete _scl;
}

void kTrans::setPos(float x, float y, float z) {
	// Change position quaternion
	_pos->set(0.0f, x, y, z);
}

void kTrans::addPos(float x, float y, float z) {
	// Increment position by given amount
	kQuat diff = kQuat(0.0f, x, y, z);
	(*_pos) = (*_pos) + diff;
}

void kTrans::setRot(float w, float x, float y, float z) {
	// Change rotation quaternion
	_rot->set(w, x, y, z);
}

void kTrans::addRot(float w, float x, float y, float z) {
	// Add new rotation after current rotation (both are VALUED)
	kQuat * newRot = new kQuat(w, x, y, z);
	newRot->convertValuedToOperational();
	_rot->convertValuedToOperational();
	_rot->set((*newRot) * (*_rot));
	_rot->convertOperationalToValued();
}

void kTrans::setScl(float x, float y, float z) {
	_scl->set(0.0f, x, y, z);
}

void kTrans::addScl(float x, float y, float z) {
	// Adjust scaling by given percentages
	_scl->vec[0] = _scl->vec[0] * x;
	_scl->vec[1] = _scl->vec[1] * y;
	_scl->vec[2] = _scl->vec[2] * z;
}

kQuat kTrans::getPos() {
	kQuat toReturn = (*_pos);
	return toReturn;
}

kQuat kTrans::getRot() {
	kQuat toReturn = (*_rot);
	return toReturn;
}

kQuat kTrans::getScl() {
	kQuat toReturn = (*_scl);
	return toReturn;
}

kTrans kTrans::operator= (kTrans operand) {
	(*_pos) = (*operand._pos);
	(*_rot) = (*operand._rot);
	(*_scl) = (*operand._scl);
	return (*this);
}

kQuat kTrans::applyTransformation(kQuat subject) {
	// Transform parent vector into child vector through frame:
	// Subtract transposition, apply rotation, and scale
	kQuat toReturn = subject - (*_pos);
	kQuat * operationalRotation = new kQuat(*_rot);
	operationalRotation->convertValuedToOperational();
	toReturn = ((*operationalRotation) * toReturn * operationalRotation->inv());
	toReturn.scale(*_scl);
	return toReturn;
}

kQuat kTrans::reverseTransformation(kQuat subject) {
	// Transform child vector into parent vector through reverse frame:
	// Reverse rotation, unscale, then add back transposition
	kQuat * rotation = new kQuat(*_rot);
	rotation->reverseValuedRotation();
	rotation->convertValuedToOperational();
	kQuat toReturn = (*rotation) * subject * rotation->inv();
	toReturn.unscale(*_scl);
	return toReturn + (*_pos);
}
	
void kTrans::glApply() {
	// Apply the transformation defined by this rotation and position
	glTranslatef(_pos->getVecI(), _pos->getVecJ(), _pos->getVecK());
	glPushMatrix();
	glRotatef(_rot->getScl() * 180.0f / float(M_PI), _rot->getVecI(), _rot->getVecJ(), _rot->getVecK());
	glScalef(_scl->vec[0], _scl->vec[1], _scl->vec[2]);
}

void kTrans::glUnapply() {
	// Unapply transformation and shift back translation
	glPopMatrix();
	glTranslatef(-1 * _pos->getVecI(), -1 * _pos->getVecJ(), -1 * _pos->getVecK());
}

void kTrans::print() {
	// Print both quaternions
	printf("Translation:       "); _pos->print();
	printf("Rotation (valued): "); _rot->print();
	printf("Scale:             "); _scl->print();
}

void kTrans::update(kQuat * linearVelocity, kQuat * angularVelocity, float dt) {
	// Position is updated linearly
	(*_pos) = (*_pos) + ((*linearVelocity) * dt);
	
	// Angular velocity and rotation must be combined operationally
	kQuat * operationalAngularVelocity = new kQuat(*angularVelocity);
	operationalAngularVelocity->scl = operationalAngularVelocity->scl * dt;
	if (angularVelocity->scl == 0.0f || angularVelocity->scl == 2 * M_PI) {
		// No angular velocity; keep old rotation
	} else if (_rot->scl == 0.0f || _rot->scl == 2 * M_PI) {
		// Frame has not rotated yet; apply one step of angular velocity
		(*_rot) = (*operationalAngularVelocity);
	} else {
		// Convert both to operational; combine; revert to valued
		operationalAngularVelocity->convertValuedToOperational();
		_rot->convertValuedToOperational();
		(*_rot) = (*operationalAngularVelocity) * (*_rot);
		_rot->convertOperationalToValued();
	}
		
	// Reduce angle to below 2 * Pi
	if (_rot->getScl() > 2 * M_PI) {
		float angle = fmod(_rot->getScl(), (float)(2 * M_PI));
		_rot->scl = angle;
	}
	
	// Normalize angle of rotation
	_rot->normalize();
}