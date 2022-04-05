/*
 *  kTrans.h
 *  kQuat
 *
 *  Created by Brian Kirkpatrick on 7/9/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 */

#ifndef KTRANS_H
#define KTRANS_H

#include "kQuat.h"

class kTrans {
private:
	kQuat * _pos; // Position of child frame. Only i, j, k are used; scl should always be 0
	kQuat * _rot; // Rotation of child frame about parent frame. Does NOT store a rotation quaternion, but rather the strict angle-and-vector rotation values
	kQuat * _scl; // Scaling of object along local axes
protected:
public:
	kTrans();
	~kTrans();
	void setPos(float x, float y, float z);
	void addPos(float x, float y, float z);
	void setRot(float w, float x, float y, float z);
	void addRot(float w, float x, float y, float z);
	void setScl(float x, float y, float z);
	void addScl(float x, float y, float z);
	kQuat getPos();
	kQuat getRot();
	kQuat getScl();
	kTrans operator= (kTrans operand);
	kQuat applyTransformation(kQuat subject);
	kQuat reverseTransformation(kQuat subject);
	void glApply();
	void glUnapply();
	void print();
	void update(kQuat * linearVelocity, kQuat * angularVelocity, float dt);
};

#endif