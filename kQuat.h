/*
 *  kQuat.h
 *  kQuat
 *
 *  Created by Brian Kirkpatrick on 5/14/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *  Defines a 3d quaternion (3d vector and a scalar) for 3d transformations
 */

#ifndef KQUAT_H
#define KQUAT_H

class kQuat {
private:
protected:
	float scl;
	float vec[3];
	friend class kTrans;
public:
	// Constructors
	kQuat();
	kQuat(const kQuat &q);
	kQuat(float x, float y, float z);
	kQuat(float s, float i, float j, float k);
	~kQuat();
	
	// Operators
	kQuat operator+ (kQuat operand);
	kQuat operator- (kQuat operand);
	kQuat operator* (float operand);
	kQuat operator* (kQuat operand);
	kQuat operator= (kQuat operand);
	
	// Methods
	kQuat conj();
	kQuat inv();
	float mag();
	void normalize();
	void scale(kQuat scaleBy);
	void unscale(kQuat scaleBy);
	void set(float w, float x, float y, float z);
	void set(kQuat newQuat);
	void print();
	float getScl() { return scl; }
	float getVecI() { return vec[0]; }
	float getVecJ() { return vec[1]; }
	float getVecK() { return vec[2]; }
	
	// Rotation
	// Note the difference between OPERATIONAL and VALUED rotational quaternion,
	// and remember that some quaternions will be merely vectors being
	// transformed by other quaternions or matrices
	void convertValuedToOperational();
	void convertOperationalToValued();
	void reverseValuedRotation();
	void reverseOperationalRotation();
};

#endif