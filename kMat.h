/*
 *  kMat.h
 *  kMat
 *
 *  Created by Brian Kirkpatrick on 2/20/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 *  Exceptions:
 *    1 - invalid index
 *    2 - incompatible dimensions
 *    3 - square matrix required
 *    4 - matrix not initialized
 *    5 - 1d matrix required
 *    6 - insufficient rank for operation
 *
 *   Note: Each matrix is a set of row vectors
 */

#ifndef K_MATRIX_H
#define K_MATRIX_H

#include "kVec.h"

class kMat {
private:
	kVec * mat;
	int r; int c;
protected:
public:
	// Constructors
	kMat();
	kMat(kMat const& b);
	kMat(kVec b);
	kMat(int size);
	kMat(int rows, int cols);
	kMat(int rows, int cols, float * vals);
	~kMat();
	
	// Access
	void set(int rows, int cols, float * vals);
	kVec toVec();
	kVec getRow(int row);
	kVec getCol(int col);
	
	// Math
	float norm();
	float trace();
	kMat I();
	kMat transpose();
	int rank();
	kMat invert();
	float det();
	
	// Operators
	friend ostream& operator <<(ostream &os, const kMat &myMat);
	friend ostream& operator <<(ostream &os, kMat *myMat);
	kVec& operator[](int i);
	kVec operator *(kVec b);
	kMat operator *(kMat b);
	kMat operator *(float b);
	kMat operator +(kMat b);
	kMat operator -(kMat b);
	kMat operator /(float b);
	kMat operator =(kMat b);
	kMat operator ^(int b);
};

#endif